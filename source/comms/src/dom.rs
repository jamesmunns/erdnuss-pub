use core::ops::DerefMut;

use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration};
use rand_core::RngCore;

use crate::{
    frame_pool::{FrameBox, RawFrameSlice},
    peer::{Peer, INCOMING_SIZE},
    CmdAddr, FrameSerial,
};

const NUM_PEERS: usize = 32;

pub enum SendError {
    NoMatchingMac,
    QueueFull,
}

pub enum RecvError {
    NoMatchingMac,
    NoMessage,
}

pub struct Dom<R: RawMutex + 'static> {
    peers: Mutex<R, [Peer; NUM_PEERS]>,
}

impl<R: RawMutex + 'static> Dom<R> {
    pub const fn uninit() -> Self {
        const ONE: Peer = Peer::const_new();
        Self {
            peers: Mutex::new([ONE; NUM_PEERS]),
        }
    }

    pub async fn init(&self, sli: &mut RawFrameSlice) {
        assert!(sli.capacity() >= (INCOMING_SIZE * NUM_PEERS));
        let mut inner = self.peers.lock().await;
        for m in inner.iter_mut() {
            let mut split = sli.split(INCOMING_SIZE).unwrap();
            core::mem::swap(sli, &mut split);
            assert_eq!(split.capacity(), INCOMING_SIZE);
            m.set_pool(split);
        }
    }

    // TODO: These shouldn't have FrameBox, they should have some other
    // type that hides the headers and stuff
    pub async fn send(&self, mac: u64, frame: FrameBox) -> Result<(), SendError> {
        let mut inner = self.peers.lock().await;
        for p in inner.iter_mut() {
            if p.is_active_mac(mac) {
                return p.enqueue_outgoing(frame).map_err(|_| SendError::QueueFull);
            }
        }
        Err(SendError::NoMatchingMac)
    }

    // TODO: These shouldn't have FrameBox, they should have some other
    // type that hides the headers and stuff
    pub async fn recv_from(&self, mac: u64) -> Result<FrameBox, RecvError> {
        let mut inner = self.peers.lock().await;
        for p in inner.iter_mut() {
            if p.is_active_mac(mac) {
                return p.dequeue_incoming().ok_or(RecvError::NoMessage);
            }
        }
        Err(RecvError::NoMatchingMac)
    }

    pub async fn step<T: FrameSerial, Rand: RngCore>(&self, serial: &mut T, rand: &mut Rand) {
        let mut inner = self.peers.lock().await;
        serve_peers(inner.deref_mut(), serial).await;
        complete_pendings(inner.deref_mut(), serial).await;
        offer_addr(inner.deref_mut(), serial, rand).await;
    }

    pub async fn connected(&self) -> heapless::Vec<u64, NUM_PEERS> {
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

async fn serve_peers<T: FrameSerial>(inner: &mut [Peer; NUM_PEERS], serial: &mut T) {
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

async fn complete_pendings<T: FrameSerial>(inner: &mut [Peer; NUM_PEERS], serial: &mut T) {
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
    inner: &mut [Peer; NUM_PEERS],
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
