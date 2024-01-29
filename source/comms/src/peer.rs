//! Peer

use crate::frame_pool::{FrameBox, RawFrameSlice};
use embassy_time::Instant;
use heapless::Deque;

/// The default number of "in-flight" packets FROM Controller TO Target
pub const OUTGOING_SIZE: usize = 8;
/// The default number of "in-flight" packets FROM Target TO Controller
pub const INCOMING_SIZE: usize = 4;

#[derive(Debug, PartialEq)]
enum State {
    Free,
    Pending,
    Active,
    Known(Instant),
}

pub(crate) struct Peer<const IN: usize = INCOMING_SIZE, const OUT: usize = OUTGOING_SIZE> {
    state: State,
    counter: u8,
    incoming_pool: RawFrameSlice,
    mac: u64,
    to_peer: Deque<FrameBox, IN>,
    from_peer: Deque<FrameBox, OUT>,
}

impl<const IN: usize, const OUT: usize> Peer<IN, OUT> {
    pub(crate) const fn const_new() -> Self {
        Self {
            state: State::Free,
            counter: 0,
            incoming_pool: RawFrameSlice::uninit(),
            mac: 0,
            to_peer: Deque::new(),
            from_peer: Deque::new(),
        }
    }

    pub(crate) fn reset_to_free(&mut self) {
        self.to_peer.clear();
        self.from_peer.clear();
        self.mac = 0;
        self.state = State::Free;
        self.counter = 0;
    }

    pub(crate) fn promote_to_active(&mut self) {
        match self.state {
            State::Pending | State::Known(_) => (),
            _ => panic!(),
        }
        // mac is already set
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Active;
        self.counter = 0;
    }

    pub(crate) fn reset_to_known(&mut self) {
        if self.state != State::Active {
            panic!();
        }
        // mac is already set
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Known(Instant::now());
        self.counter = 0;
    }

    pub(crate) fn promote_to_known_with_mac(&mut self, mac: u64) {
        if self.state != State::Free {
            panic!();
        }
        self.mac = mac;
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Known(Instant::now());
        self.counter = 0;
    }

    pub(crate) fn promote_to_pending(&mut self, mac: u64) {
        if self.state != State::Free {
            panic!();
        }
        self.mac = mac;
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Pending;
        self.counter = 0;
    }

    pub(crate) fn set_success(&mut self) {
        self.counter = 0;
    }

    pub(crate) fn increment_error(&mut self) {
        match self.state {
            State::Free => {
                // uh?
            }
            State::Known(_) => {
                // We currently ignore errors while in the known state. This may or may not be a mistake
            }
            State::Pending => {
                // one strike, you're out!
                self.reset_to_free();
            }
            State::Active => {
                // TODO: We should probably drop all incoming/outgoing messages
                // in the deques. We may ALSO want to hold this address as "unusable"
                // for some amount of time, to ensure we don't re-use the address before
                // the Target "notices" it has been dropped, to avoid a flaky device from
                // incorrectly "sharing" the logical address with a new device.
                //
                // We might want a separate "timeout/inhibit" state that is used when
                // moving from Active -> Free with a timestamp.
                self.counter += 1;
                if self.counter > 3 {
                    nut_warn!("Resetting active device to known");
                    self.reset_to_known();
                }
            }
        }
    }

    #[inline]
    pub(crate) fn is_pending(&self) -> Option<u64> {
        if self.state == State::Pending {
            Some(self.mac)
        } else {
            None
        }
    }

    #[inline]
    pub(crate) fn is_active(&self) -> bool {
        self.state == State::Active
    }

    #[inline]
    pub(crate) fn is_known(&self) -> Option<(u64, Instant)> {
        if let State::Known(instant) = self.state {
            Some((self.mac, instant))
        } else {
            None
        }
    }

    #[inline]
    pub(crate) fn mac(&self) -> u64 {
        self.mac
    }

    #[inline]
    pub(crate) fn is_idle(&self) -> bool {
        if self.state != State::Free {
            return false;
        }
        self.incoming_pool.count_allocatable() == INCOMING_SIZE
    }

    pub(crate) fn alloc_incoming(&mut self) -> Option<FrameBox> {
        self.incoming_pool.allocate_raw()
    }

    #[inline]
    pub(crate) fn is_active_mac(&self, mac: u64) -> bool {
        if self.state != State::Active {
            return false;
        }
        self.mac == mac
    }

    #[inline]
    pub(crate) fn enqueue_incoming(&mut self, msg: FrameBox) {
        // The deque length is the same as the pool size,
        // so this should never fail.
        self.from_peer.push_front(msg).map_err(drop).unwrap();
    }

    #[inline]
    pub(crate) fn enqueue_outgoing(&mut self, msg: FrameBox) -> Result<(), FrameBox> {
        self.to_peer.push_front(msg)
    }

    #[inline]
    pub(crate) fn dequeue_incoming(&mut self) -> Option<FrameBox> {
        self.from_peer.pop_back()
    }

    #[inline]
    pub(crate) fn dequeue_outgoing(&mut self) -> Option<FrameBox> {
        self.to_peer.pop_back()
    }

    pub(crate) fn set_pool(&mut self, pool: RawFrameSlice) {
        self.incoming_pool = pool;
    }
}
