//! Controller Interface
//!
//! The Controller is responsible for running the bus.

use core::{fmt::Debug, ops::DerefMut};

use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration, TimeoutError, Instant};
use rand_core::RngCore;

use crate::{
    frame_pool::{FrameBox, RawFrameSlice, SendFrameBox, WireFrameBox},
    peer::{Peer, INCOMING_SIZE, OUTGOING_SIZE},
    CmdAddr, Error, FrameSerial, MAX_TARGETS,
};

/// Time that a Controller will wait for a Target to respond
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(1);

/// Time that a peer can be in the Known state before getting reset to Free
pub const KNOWN_TIMEOUT: Duration = Duration::from_secs(5);

/// Controller interface and data storage
///
/// The static Controller is intended to be used in two separate places
/// in an application:
///
/// 1. In one place, where [Controller::step()] is called periodically, to
///    service bus operations
/// 2. In another place, where [Controller::recv_from()] or [Controller::send()]
///    are called, to process or forward messages to/from the bus
///
/// In a typical example where the Controller is acting as a Bridge/Router over
/// USB, place 1. would be a standalone task, calling step at some periodic polling
/// rate, and place 2. would be grouped with the USB interface for sending/receiving
/// frames over USB.
///
/// The Controller contains an async Mutex which provides interior mutable access to
/// information such as the table of connected Targets, or any "in flight" messages.
pub struct Controller<
    R: RawMutex + 'static,
    const IN: usize = INCOMING_SIZE,
    const OUT: usize = OUTGOING_SIZE,
> {
    peers: Mutex<R, [Peer<IN, OUT>; MAX_TARGETS]>,
}

/// Instantiation and Initialization methods
impl<R: RawMutex + 'static, const IN: usize, const OUT: usize> Controller<R, IN, OUT> {
    const ONE: Peer<IN, OUT> = Peer::<IN, OUT>::const_new();

    /// Create a new, uninitialized controller structure
    ///
    /// Intended to be used to create as a static to avoid large
    /// stack initializations.
    ///
    /// Users must still call [`Controller::init()`] before use.
    ///
    /// ```rust
    /// use erdnuss_comms::controller::Controller;
    /// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    ///
    /// // NOTE: you can use any mutex suitable for you, such as `ThreadModeRawMutex`
    /// // if you not using this from an interrupt context
    /// static CONTROLLER: Controller<CriticalSectionRawMutex> = Controller::uninit();
    /// ```
    pub const fn uninit() -> Controller<R, IN, OUT> {
        Self {
            peers: Mutex::new([Self::ONE; MAX_TARGETS]),
        }
    }

    /// Initialize the [Controller]
    ///
    /// This initialization provides the backing storage for the incoming target
    /// frames. [RawFrameSlice] must contain AT LEAST `IN` times `OUT`
    /// frame storage slots. `sli`'s capacity will be reduced by this amount.
    pub async fn init(&self, sli: &mut RawFrameSlice) {
        assert!(sli.capacity() >= (INCOMING_SIZE * MAX_TARGETS));
        let mut inner = self.peers.lock().await;
        for m in inner.iter_mut() {
            let mut split = sli.split(INCOMING_SIZE).unwrap();
            core::mem::swap(sli, &mut split);
            assert_eq!(split.capacity(), INCOMING_SIZE);
            m.set_pool(split);
        }
    }
}

/// Bus management and operation method(s)
impl<R: RawMutex + 'static> Controller<R> {
    /// Perform one "step" of the bus
    ///
    /// One call to `step` will:
    ///
    /// 1. Send UP TO one message to any known Targets, and Receive
    ///    UP TO one message from any known Target
    /// 2. Attempt to complete any pending logical address offers
    /// 3. Attempt to offer UP TO one unused logical address
    ///
    /// This method should be called regularly.
    ///
    /// The `serial` and `rand` resources are passed in on every call to `step`,
    /// rather than making them part of the entire `Controller` entity for a couple
    /// of stylistic reasons:
    ///
    /// * Keep the static type-generics less complex
    /// * Make initialization less complex
    /// * Make these generics NOT required for the shared `send`/`recv_from`
    ///   interface methods
    ///
    /// The inner async mutex will be locked for the entire duration of the call to
    /// `step`, which may be held for some amount of time, depending on the number of
    /// bus timeouts and total amount of data transferred. This may be on the order of
    /// millisecond(s).
    pub async fn step<T, Rand>(
        &self,
        serial: &mut T,
        rand: &mut Rand,
    ) -> Result<(), Error<T::SerError>>
    where
        T: FrameSerial,
        Rand: RngCore,
    {
        let mut inner = self.peers.lock().await;
        serve_peers(inner.deref_mut(), serial).await?;
        complete_pendings(inner.deref_mut(), serial).await?;
        update_known(inner.deref_mut(), serial).await?;
        offer_addr(inner.deref_mut(), serial, rand).await?;
        Ok(())
    }
}

/// Bus I/O methods
impl<R: RawMutex + 'static> Controller<R> {
    /// Attempt to enqueue a message for sending
    pub async fn send(&self, mac: u64, frame: SendFrameBox) -> Result<(), SendError> {
        self.peers
            .lock()
            .await
            .iter_mut()
            .find(|p| p.is_active_mac(mac))
            .ok_or(SendError::NoMatchingMac)
            .and_then(|p| {
                p.enqueue_outgoing(frame.into_inner())
                    .map_err(SendError::QueueFull)
            })
    }

    /// Attempt to receive a message from the given unique address
    pub async fn recv_from(&self, mac: u64) -> Result<WireFrameBox, RecvError> {
        self.peers
            .lock()
            .await
            .iter_mut()
            .find(|p| p.is_active_mac(mac))
            .ok_or(RecvError::NoMatchingMac)
            .and_then(|p| p.dequeue_incoming().ok_or(RecvError::NoMessage))
            .map(WireFrameBox::new_unchecked)
    }

    /// Get a list of all target devices on the bus
    ///
    /// This list DOES NOT include the Controller's MAC address, but the returned
    /// [`heapless::Vec`] *does* reserve enough room to contain all [MAX_TARGETS]
    /// plus one.
    pub async fn connected(&self) -> heapless::Vec<u64, { MAX_TARGETS + 1 }> {
        self.peers
            .lock()
            .await
            .iter()
            .filter_map(|p| p.is_active().then_some(p.mac()))
            .collect()
    }

    /// Adds a list of macs to the peers as known peers
    pub async fn add_known_macs(&self, mut macs: heapless::Vec<u64, { MAX_TARGETS }>) {
        self.peers
            .lock()
            .await
            .iter_mut()
            .for_each(|p| {
                if !macs.is_empty() && p.is_idle() {
                    let mac = macs.pop().expect("No mac to pop");
                    p.promote_to_known_with_mac(mac);
                }
            })
    }
}

/// An error when sending a frame to a Target
pub enum SendError {
    /// Attempted to send to an unknown MAC address
    NoMatchingMac,
    /// The given MAC address was known, but the outgoing queue
    /// of this device is full
    QueueFull(FrameBox),
}

impl Debug for SendError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("SendError::")?;
        let remain = match self {
            SendError::NoMatchingMac => "NoMatchingMac",
            SendError::QueueFull(_) => "QueueFull(...)",
        };
        f.write_str(remain)
    }
}

/// An error when attempting to receive a frame from a Target
#[derive(Debug, PartialEq)]
pub enum RecvError {
    /// Attempted to receive from an unknown MAC address
    NoMatchingMac,
    /// The given MAC address was known, but the incoming queue
    /// of this device was empty
    NoMessage,
}

/// A helper function that serves all currently active peers, exchanging
/// zero or one frames in each direction
async fn serve_peers<T: FrameSerial>(
    inner: &mut [Peer; MAX_TARGETS],
    serial: &mut T,
) -> Result<(), Error<T::SerError>> {
    // First pass: poll all active devices
    for (i, p) in inner.iter_mut().enumerate() {
        // We only care about active devices
        if !p.is_active() {
            continue;
        }

        // Can we allocate a reception frame? If not: we can't talk to the device, skip it
        // for a single round. In the future, we might want to increment error here to
        // eventually time out devices, but this isn't really the fault of the target, it's
        // a fault of the firmware driving the "controller".
        let Some(mut rx) = p.alloc_incoming() else {
            nut_warn!("Couldn't alloc incoming!");
            p.increment_error();
            continue;
        };

        // Is there any outgoing frame? If not, we use a one byte fallback buffer
        // to place the "Select" command in.
        let mut maybe_out = p.dequeue_outgoing();
        let mut fallback = [0u8; 1];
        let to_send = match maybe_out.as_deref_mut() {
            Some(fb) => fb,
            None => &mut fallback,
        };

        // Fill in the cmdaddr, send the message, and start listening with a
        // timeout
        to_send[0] = CmdAddr::SelectAddr(i as u8).into();
        serial.send_frame(to_send).await?;
        let rxto = with_timeout(REPLY_TIMEOUT, serial.recv(&mut rx));

        match rxto.await {
            Ok(Ok(tf)) => {
                // We received a message within the timeout!
                let len = tf.frame.len();
                if len != 0 && tf.frame[0] == CmdAddr::ReplyFromAddr(i as u8).into() {
                    // We got AT least an ack, mark that as a success
                    p.set_success();

                    // If there was some kind of body, pass it on
                    if len > 1 {
                        nut_trace!("Got msg len {=usize} for {=usize}", len, i);
                        rx.set_len(len);
                        p.enqueue_incoming(rx);
                    }
                } else {
                    // We got a zero len message, OR an unexpected reply. Mark an error.
                    nut_warn!("Error with {=usize} len is {=usize}", i, len);
                    p.increment_error();
                }
            }
            Ok(Err(e)) => {
                // We finished within the timeout, but got some kind of error
                // while receiving. Increment the error, in case we don't just
                // decide to reset or something.
                p.increment_error();

                // then bubble up the error.
                return Err(e);
            }
            Err(TimeoutError) => {
                // We timed out, increment error
                p.increment_error();
            }
        }
    }
    Ok(())
}

/// A helper function for moving targets from the Pending stage to the Active stage
async fn complete_pendings<T: FrameSerial>(
    inner: &mut [Peer; MAX_TARGETS],
    serial: &mut T,
) -> Result<(), Error<T::SerError>> {
    for (i, p) in inner.iter_mut().enumerate() {
        // Only worry about pending nodes
        let Some(mac) = p.is_pending() else {
            continue;
        };

        // Send a message with the expected MAC address for confirmation
        let mut out_buf = [0u8; 9];
        out_buf[0] = CmdAddr::DiscoverySuccess(i as u8).into();
        out_buf[1..9].copy_from_slice(&mac.to_le_bytes());

        // We should only get back an empty ACK and nothing else
        let mut in_buf = [0u8; 2];
        serial.send_frame(&out_buf).await?;
        let rxto = with_timeout(REPLY_TIMEOUT, serial.recv(&mut in_buf));

        match rxto.await {
            Ok(Ok(tf)) => {
                let frame = tf.frame;
                let good_len = frame.len() == 1;
                let good_hdr = good_len && frame[0] == CmdAddr::ReplyFromAddr(i as u8).into();
                if good_hdr {
                    nut_info!("Promoting to active {=usize} {=u64}", i, mac);
                    p.promote_to_active();
                } else {
                    p.increment_error();
                }
            }
            Ok(Err(_e)) => {
                // We got some kind of receive error, just mark this as
                // an error and move on
                p.increment_error();
                continue;
            }
            Err(TimeoutError) => {
                // No answer? No address.
                p.increment_error();
            }
        }
    }
    Ok(())
}

/// A helper function for moving targets from the Known stage to either the Active stage or to the Free stage
async fn update_known<T: FrameSerial>(
    inner: &mut [Peer; MAX_TARGETS],
    serial: &mut T,
) -> Result<(), Error<T::SerError>> {
    for (i, p) in inner.iter_mut().enumerate() {
        // Only worry about pending nodes
        let Some((mac, instant)) = p.is_known() else {
            continue;
        };

        if instant + KNOWN_TIMEOUT >= Instant::now() {
            p.reset_to_free();
            continue;
        }

        // Send a message with the expected MAC address for confirmation
        let mut out_buf = [0u8; 9];
        out_buf[0] = CmdAddr::DiscoverySuccess(i as u8).into();
        out_buf[1..9].copy_from_slice(&mac.to_le_bytes());

        // We should only get back an empty ACK and nothing else
        let mut in_buf = [0u8; 2];
        serial.send_frame(&out_buf).await?;
        let rxto = with_timeout(REPLY_TIMEOUT, serial.recv(&mut in_buf));

        match rxto.await {
            Ok(Ok(tf)) => {
                let frame = tf.frame;
                let good_len = frame.len() == 1;
                let good_hdr = good_len && frame[0] == CmdAddr::ReplyFromAddr(i as u8).into();
                if good_hdr {
                    nut_info!("Promoting to active {=usize} {=u64}", i, mac);
                    p.promote_to_active();
                } else {
                    p.increment_error();
                }
            }
            Ok(Err(_e)) => {
                // We got some kind of receive error, just mark this as
                // an error and move on
                p.increment_error();
                continue;
            }
            Err(TimeoutError) => {
                // No answer? No address.
                p.increment_error();
            }
        }
    }
    Ok(())
}

/// A helper function for moving new nodes into the Pending stage
async fn offer_addr<T: FrameSerial, R: RngCore>(
    inner: &mut [Peer; MAX_TARGETS],
    serial: &mut T,
    rand: &mut R,
) -> Result<(), Error<T::SerError>> {
    let Some((i, p)) = inner.iter_mut().enumerate().find(|(_i, p)| p.is_idle()) else {
        return Ok(());
    };

    // We found an idle slot! Offer it up.
    let mut out_buf = [0u8; 9];
    out_buf[0] = CmdAddr::DiscoveryOffer(i as u8).into();
    rand.fill_bytes(&mut out_buf[1..9]);
    serial.send_frame(&out_buf).await?;

    let mut in_buf = [0u8; 10];
    let rxto = with_timeout(Duration::from_millis(1), serial.recv(&mut in_buf));
    match rxto.await {
        Ok(Ok(tf)) => {
            let frame = tf.frame;
            let good_len = frame.len() == 9;
            let good_hdr = good_len && frame[0] == CmdAddr::DiscoveryClaim(i as u8).into();

            if good_hdr {
                let mut mac = [0u8; 8];
                let rand_iter = out_buf[1..9].iter();
                let resp_iter = frame[1..9].iter();

                mac.iter_mut()
                    .zip(rand_iter.zip(resp_iter))
                    .for_each(|(d, (a, b))| *d = *a ^ *b);


                // If the mac is known in the list already it will time out soonish anyway
                p.promote_to_pending(u64::from_le_bytes(mac));
            }
        }
        Ok(Err(e)) => return Err(e),
        Err(_) => {
            // Timed out, no one wanted the address
        }
    }

    Ok(())
}
