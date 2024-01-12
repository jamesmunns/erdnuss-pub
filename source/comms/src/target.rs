use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::{with_timeout, Duration, Timer};
use futures::FutureExt;
use rand_core::RngCore;

use crate::{
    frame_pool::{FrameBox, RawFrameSlice},
    CmdAddr, FrameSerial,
};

pub trait SubCfg {
    type Mutex: RawMutex + 'static;
    type Serial: FrameSerial;
    type Rand: RngCore;
}

pub struct Sub<'a, Cfg, const BUFS: usize>
where
    Cfg: SubCfg,
{
    serial: Cfg::Serial,
    to_app: Sender<'a, Cfg::Mutex, FrameBox, BUFS>,
    from_app: Receiver<'a, Cfg::Mutex, FrameBox, BUFS>,
    pool: RawFrameSlice,
    mac: [u8; 8],
    rand: Cfg::Rand,
}

impl<'a, Cfg, const BUFS: usize> Sub<'a, Cfg, BUFS>
where
    Cfg: SubCfg,
{
    pub fn new(
        serial: Cfg::Serial,
        to_app: Sender<'a, Cfg::Mutex, FrameBox, BUFS>,
        from_app: Receiver<'a, Cfg::Mutex, FrameBox, BUFS>,
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
        }
    }

    pub async fn run(&mut self) {
        'outer: loop {
            let addr = self.get_addr().await;
            defmt::println!("Got addr: {=u8}", addr);

            loop {
                match with_timeout(Duration::from_secs(3), self.exchange_one(addr)).await {
                    // Exchange happened w/in timeout
                    Ok(Ok(())) => {}
                    // Exchange happened w/in timeout, but errored
                    Ok(Err(_)) => {
                        defmt::println!("Error :(");
                        continue 'outer;
                    }
                    // Timed out
                    Err(_) => {
                        defmt::println!("Timed out!");
                        continue 'outer;
                    }
                }
            }
        }
    }

    async fn exchange_one(
        &mut self,
        addr: u8,
    ) -> Result<(), crate::Error<<Cfg::Serial as FrameSerial>::SerError>> {
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
        Timer::at(time + Duration::from_micros(25)).await;
        self.serial.send_frame(out).await?;
        Ok(())
    }

    async fn get_incoming(
        &mut self,
        addr: u8,
    ) -> Result<crate::Instant, crate::Error<<Cfg::Serial as FrameSerial>::SerError>> {
        let mut frame = self.pool.allocate_raw().unwrap();
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
            defmt::println!("get_addr...");
            let goforit = self.rand.next_u32();

            // Wait for an offer frame
            let (offer_addr, offer_challenge) = self.get_offer().await;

            // do we go for it? (1/8 chance)
            if goforit & 0b0000_0111 != 0 {
                defmt::println!("skipping!");
                continue;
            } else {
                defmt::println!("going for it!");
            }

            let claim_dance = async {
                self.send_claim(offer_addr, &offer_challenge).await?;
                self.get_success(offer_addr).await?;
                let msg: [u8; 1] = [CmdAddr::ReplyFromAddr(offer_addr).into()];
                self.serial.send_frame(&msg).await?;
                Result::<(), crate::Error<<Cfg::Serial as FrameSerial>::SerError>>::Ok(())
            };

            // Give ourselves some time to complete, if not try again
            match with_timeout(Duration::from_secs(3), claim_dance).await {
                Ok(Ok(())) => return offer_addr,
                _ => continue,
            }
        }
    }

    async fn get_success(
        &mut self,
        offer_addr: u8,
    ) -> Result<(), crate::Error<<Cfg::Serial as FrameSerial>::SerError>> {
        // A success packet should be 9 bytes plus one extra for
        // the line break.
        let mut scratch = [0u8; 16];
        loop {
            defmt::println!("get success");
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
    ) -> Result<(), crate::Error<<Cfg::Serial as FrameSerial>::SerError>> {
        defmt::println!("send claim");
        let mut claim = [0u8; 9];

        claim[0] = CmdAddr::DiscoveryClaim(offer_addr).into();
        let data = &mut claim[1..9];
        data.copy_from_slice(&self.mac);

        data.iter_mut()
            .zip(challenge.iter())
            .for_each(|(a, b)| *a ^= *b);

        self.serial.send_frame(&claim).await
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
}
