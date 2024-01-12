//! Controller Interface
//!
//! The Controller is responsible for running the bus.

use core::ops::DerefMut;

use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration};
use rand_core::RngCore;

use crate::{
    frame_pool::{FrameBox, RawFrameSlice},
    peer::{Peer, INCOMING_SIZE},
    wirehelp::{WireFrameBox, SendFrameBox},
    CmdAddr, FrameSerial, MAX_TARGETS,
};

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
pub struct Controller<R: RawMutex + 'static> {
    peers: Mutex<R, [Peer; MAX_TARGETS]>,
}

/// Instantiation and Initialization methods
impl<R: RawMutex + 'static> Controller<R> {
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
    pub const fn uninit() -> Self {
        const ONE: Peer = Peer::const_new();
        Self {
            peers: Mutex::new([ONE; MAX_TARGETS]),
        }
    }

    /// Initialize the [Controller]
    ///
    /// This initialization provides the backing storage for the incoming target
    /// frames. [RawFrameSlice] must contain AT LEAST [INCOMING_SIZE] times [MAX_TARGETS]
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
    pub async fn step<T, Rand>(&self, serial: &mut T, rand: &mut Rand)
    where
        T: FrameSerial,
        Rand: RngCore
    {
        let mut inner = self.peers.lock().await;
        serve_peers(inner.deref_mut(), serial).await;
        complete_pendings(inner.deref_mut(), serial).await;
        offer_addr(inner.deref_mut(), serial, rand).await;
    }
}

/// Bus I/O methods
impl<R: RawMutex + 'static> Controller<R> {
    // TODO: These shouldn't have FrameBox, they should have some other
    // type that hides the headers and stuff
    pub async fn send(&self, mac: u64, frame: SendFrameBox) -> Result<(), SendError> {
        let mut inner = self.peers.lock().await;
        for p in inner.iter_mut() {
            if p.is_active_mac(mac) {
                return p.enqueue_outgoing(frame.into_inner()).map_err(|_| SendError::QueueFull);
            }
        }
        Err(SendError::NoMatchingMac)
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



    pub async fn connected(&self) -> heapless::Vec<u64, MAX_TARGETS> {
        let mut out = heapless::Vec::new();
        let inner = self.peers.lock().await;
        for p in inner.iter() {
            if p.is_active() {
                let _ = out.push(p.mac());
            }
        }
        out
    }
}

pub enum SendError {
    NoMatchingMac,
    QueueFull,
}

pub enum RecvError {
    NoMatchingMac,
    NoMessage,
}

async fn serve_peers<T: FrameSerial>(inner: &mut [Peer; MAX_TARGETS], serial: &mut T) {
    // First pass: poll all active devices
    for (i, p) in inner.iter_mut().enumerate() {
        if !p.is_active() {
            continue;
        }
        // Can we allocate a reception frame?
        let Some(mut rx) = p.alloc_incoming() else {
            defmt::println!("Couldn't alloc incoming!");
            p.increment_error();
            continue;
        };
        let mut maybe_out = p.dequeue_outgoing();
        let mut fallback = [0u8; 1];
        let to_send = match maybe_out.as_deref_mut() {
            Some(fb) => fb,
            None => &mut fallback,
        };

        to_send[0] = CmdAddr::SelectAddr(i as u8).into();
        serial.send_frame(to_send).await.map_err(drop).unwrap();
        let rxto = with_timeout(Duration::from_millis(1), serial.recv(&mut rx));

        match rxto.await {
            Ok(Ok(tf)) => {
                let len = tf.frame.len();
                if len != 0 && tf.frame[0] == CmdAddr::ReplyFromAddr(i as u8).into() {
                    if len > 1 {
                        defmt::println!("Got msg len {=usize} for {=usize}", len, i);
                        rx.set_len(len);
                        p.enqueue_incoming(rx);
                        p.set_success();
                    }
                } else {
                    defmt::println!("Error with {=usize} len is {=usize}", i, len);
                    p.increment_error();
                }
            }
            Ok(Err(_e)) => continue,
            Err(_) => {
                // timed out
                p.increment_error();
            }
        }
    }
}

async fn complete_pendings<T: FrameSerial>(inner: &mut [Peer; MAX_TARGETS], serial: &mut T) {
    for (i, p) in inner.iter_mut().enumerate() {
        let Some(mac) = p.is_pending() else {
            continue;
        };
        let mut out_buf = [0u8; 9];
        out_buf[0] = CmdAddr::DiscoverySuccess(i as u8).into();
        out_buf[1..9].copy_from_slice(&mac.to_le_bytes());
        let mut in_buf = [0u8; 2];
        serial.send_frame(&out_buf).await.map_err(drop).unwrap();
        let rxto = with_timeout(Duration::from_millis(1), serial.recv(&mut in_buf));
        match rxto.await {
            Ok(Ok(tf)) => {
                let frame = tf.frame;
                let good_len = frame.len() == 1;
                let good_hdr = good_len && frame[0] == CmdAddr::ReplyFromAddr(i as u8).into();
                if good_hdr {
                    defmt::println!("Promoting to active {=usize} {=u64}", i, mac);
                    p.promote_to_active();
                } else {
                    p.increment_error();
                }
            }
            Ok(Err(_e)) => continue,
            Err(_) => {
                p.increment_error();
            }
        }
    }
}

async fn offer_addr<T: FrameSerial, R: RngCore>(
    inner: &mut [Peer; MAX_TARGETS],
    serial: &mut T,
    rand: &mut R,
) {
    for (i, p) in inner.iter_mut().enumerate() {
        if !p.is_idle() {
            continue;
        }

        // We found an idle slot! Offer it up.
        let mut out_buf = [0u8; 9];
        out_buf[0] = CmdAddr::DiscoveryOffer(i as u8).into();
        rand.fill_bytes(&mut out_buf[1..9]);
        let mut in_buf = [0u8; 10];
        serial.send_frame(&out_buf).await.map_err(drop).unwrap();

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

                    p.promote_to_pending(u64::from_le_bytes(mac));
                }
            }
            Ok(Err(_e)) => return,
            Err(_) => {
                // Timed out, no one wanted the address
            }
        }

        // Only offer one address per round.
        return;
    }
}
