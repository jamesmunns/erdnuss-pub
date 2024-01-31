//! Target interface
//!
//! This interface is used when operating as a Target.

use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex}, channel::{Receiver, Sender}, signal::Signal
};
use embassy_time::{with_timeout, Duration, Timer};
use futures::FutureExt;
use rand_core::RngCore;

use crate::{
    frame_pool::{FrameBox, RawFrameSlice}, CmdAddr, FrameSerial
};

/// The default number of "in-flight" packets FROM Target TO Controller
pub const OUTGOING_SIZE: usize = 8;
/// The default number of "in-flight" packets FROM Controller TO Target
pub const INCOMING_SIZE: usize = 4;

/// Metadata trait to contain relevant generics
pub trait TgtCfg {
    /// Mutex type used for channels
    type Mutex: RawMutex + 'static;

    /// Serial interface type
    type Serial: FrameSerial;

    /// Random number generator
    type Rand: RngCore;

    /// Amount of time to delay from hearing a response to
    /// sending a reply.
    const TURNAROUND_DELAY: Duration;

    /// Amount of time from initiating a claim to getting an address
    const ADDRESS_CLAIM_TIMEOUT: Duration;

    /// Amount of time being unaddressed before trying to get a new
    /// address
    const SELECT_TIMEOUT: Duration;
}

enum TargetError<S> {
    Serial(S),
    Oom,
}

impl<S> From<crate::Error<S>> for TargetError<S> {
    fn from(value: crate::Error<S>) -> Self {
        match value {
            crate::Error::Serial(s) => Self::Serial(s),
        }
    }
}

/// Enum of possible states a target can have
#[derive(Debug, Clone, Copy)]
pub enum State {
    /// The target is connected (with the current address)
    Connected(u8),
    /// The target is disconnected
    Disconnected,
}

/// Interface for the Target
///
/// Note that UNLIKE the [`Controller`][crate::Controller], which uses a Mutex to share between the
/// "application facing" and "wire facing" parts, we instead use a pair of
/// [`Channel`][embassy_sync::channel::Channel]s for the Target instead.
///
/// This is because the Target is MUCH more timing critical, holding a mutex locked when addressed
/// by the Controller could cause us to totally miss a message.
pub struct Target<'a, Cfg, const IN: usize = INCOMING_SIZE, const OUT: usize = OUTGOING_SIZE>
where
    Cfg: TgtCfg,
{
    serial: Cfg::Serial,
    to_app: Sender<'a, Cfg::Mutex, FrameBox, IN>,
    from_app: Receiver<'a, Cfg::Mutex, FrameBox, OUT>,
    pool: RawFrameSlice,
    mac: [u8; 8],
    rand: Cfg::Rand,
    state: Signal<CriticalSectionRawMutex, State>
}

impl<'a, Cfg, const IN: usize, const OUT: usize> Target<'a, Cfg, IN, OUT>
where
    Cfg: TgtCfg,
{
    /// Create a new [Target] worker.
    pub fn new(
        serial: Cfg::Serial,
        to_app: Sender<'a, Cfg::Mutex, FrameBox, IN>,
        from_app: Receiver<'a, Cfg::Mutex, FrameBox, OUT>,
        pool: RawFrameSlice,
        mac: [u8; 8],
        rand: Cfg::Rand,
    ) -> Self {
        Self {
            serial,
            to_app,
            from_app,
            mac,
            rand,
            pool,
            state: Signal::default()
        }
    }

    /// Run forever, exchanging messages
    pub async fn run(&mut self) {
        'outer: loop {
            self.state.signal(State::Disconnected);
            let addr = self.get_addr().await;
            nut_info!("Got addr: {=u8}", addr);
            self.state.signal(State::Connected(addr));

            loop {
                match with_timeout(Cfg::SELECT_TIMEOUT, self.exchange_one(addr)).await {
                    // Exchange happened w/in timeout
                    Ok(Ok(())) => {}
                    // Exchange happened w/in timeout, but errored
                    Ok(Err(_)) => {
                        nut_error!("Error :(");
                        continue 'outer;
                    }
                    // Timed out
                    Err(_) => {
                        nut_warn!("Timed out!");
                        continue 'outer;
                    }
                }
            }
        }
    }

    async fn exchange_one(
        &mut self,
        addr: u8,
    ) -> Result<(), TargetError<<Cfg::Serial as FrameSerial>::SerError>> {
        // Wait for us to be acknowledged, and pass on the frame if we get one
        let time = self.get_incoming(addr).await?;

        // Is there something to send now? If not, empty-ack.
        let mut tx_frame = self.from_app.receive().now_or_never();
        let mut fallback = [0u8; 1];
        let out = match tx_frame.as_deref_mut() {
            Some(g) => g,
            None => fallback.as_mut_slice(),
        };
        out[0] = CmdAddr::ReplyFromAddr(addr).into();

        // Send reply
        Timer::at(time + Cfg::TURNAROUND_DELAY).await;
        self.serial.send_frame(out).await?;
        Ok(())
    }

    async fn get_incoming(
        &mut self,
        addr: u8,
    ) -> Result<crate::Instant, TargetError<<Cfg::Serial as FrameSerial>::SerError>> {
        let mut frame = self.pool.allocate_raw().ok_or(TargetError::Oom)?;
        loop {
            let buf = &mut frame[..];
            let got = self.serial.recv(buf).await?;
            if got.frame.is_empty() {
                continue;
            }
            let Ok(cmd_addr) = CmdAddr::try_from(got.frame[0]) else {
                continue;
            };
            if cmd_addr != CmdAddr::SelectAddr(addr) {
                continue;
            }
            let len = got.frame.len();
            let stamp = got.end_of_rx;

            if len != 1 {
                frame.set_len(len);
                self.to_app.send(frame).await;
            }
            return Ok(stamp);
        }
    }

    async fn get_addr(&mut self) -> u8 {
        loop {
            nut_info!("get_addr...");
            let goforit = self.rand.next_u32();

            // Wait for an offer frame
            let (offer_addr, offer_challenge) = self.get_offer().await;

            // do we go for it? (1/8 chance)
            if goforit & 0b0000_0111 != 0 {
                nut_info!("skipping!");
                continue;
            } else {
                nut_info!("going for it!");
            }

            let claim_dance = async {
                self.send_claim(offer_addr, &offer_challenge).await?;
                self.get_success(offer_addr).await?;
                let msg: [u8; 1] = [CmdAddr::ReplyFromAddr(offer_addr).into()];
                self.serial.send_frame(&msg).await?;
                Result::<(), TargetError<<Cfg::Serial as FrameSerial>::SerError>>::Ok(())
            };

            // Give ourselves some time to complete, if not try again
            match with_timeout(Cfg::ADDRESS_CLAIM_TIMEOUT, claim_dance).await {
                Ok(Ok(())) => return offer_addr,
                _ => continue,
            }
        }
    }

    async fn get_success(
        &mut self,
        offer_addr: u8,
    ) -> Result<(), TargetError<<Cfg::Serial as FrameSerial>::SerError>> {
        // A success packet should be 9 bytes plus one extra for
        // the line break.
        let mut scratch = [0u8; 16];
        loop {
            nut_info!("get success");
            let Ok(tframe) = self.serial.recv(&mut scratch).await else {
                // log: wat
                // NOTE: I feel like this is probably going to be due to
                // hearing messages longer than `scratch`.
                continue;
            };
            // Is this a valid cmd addr?
            let Some(ca) = tframe
                .frame
                .first()
                .and_then(|b| CmdAddr::try_from(*b).ok())
            else {
                continue;
            };
            // Is this an offer?
            let CmdAddr::DiscoverySuccess(addr) = ca else {
                continue;
            };

            // Is this long enough?
            if tframe.frame.len() < 9 {
                continue;
            }

            // Is this for us?
            if addr == offer_addr && (tframe.frame[1..9] == self.mac) {
                return Ok(());
            } else {
                continue;
            }
        }
    }

    async fn send_claim(
        &mut self,
        offer_addr: u8,
        challenge: &[u8; 8],
    ) -> Result<(), TargetError<<Cfg::Serial as FrameSerial>::SerError>> {
        nut_info!("send claim");
        let mut claim = [0u8; 9];

        claim[0] = CmdAddr::DiscoveryClaim(offer_addr).into();
        let data = &mut claim[1..9];
        data.copy_from_slice(&self.mac);

        data.iter_mut()
            .zip(challenge.iter())
            .for_each(|(a, b)| *a ^= *b);

        self.serial.send_frame(&claim).await?;
        Ok(())
    }

    async fn get_offer(&mut self) -> (u8, [u8; 8]) {
        // offer should be 1 + 8 + 1 for line break
        let mut scratch = [0u8; 16];
        loop {
            let Ok(tframe) = self.serial.recv(&mut scratch).await else {
                // log: wat
                // probably due to scratch being too small if we hear
                // someone elses frame
                continue;
            };
            // Is this a valid cmd addr?
            let Some(ca) = tframe
                .frame
                .first()
                .and_then(|b| CmdAddr::try_from(*b).ok())
            else {
                continue;
            };
            // Is this an offer?
            let CmdAddr::DiscoveryOffer(addr) = ca else {
                continue;
            };
            // Is this long enough?
            if tframe.frame.len() < 9 {
                continue;
            }
            let mut challenge = [0u8; 8];
            challenge.copy_from_slice(&tframe.frame[1..9]);

            return (addr, challenge);
        }
    }

    /// wait for a change in the targets state
    pub async fn state_change(&self) -> State {
        self.state.wait().await
    }


    // TODO: Not possible until dependency embassy-sync is updated
    // /// Get the targets current state (or None if for whatever reason no state is set)
    // pub fn get_state(&self) -> Option<State> {
    //     self.state.try_take()
    // }
}
