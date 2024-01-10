use crate::frame_pool::{FrameBox, RawFrameSlice};
use heapless::Deque;

pub const OUTGOING_SIZE: usize = 8;
pub const INCOMING_SIZE: usize = 4;

#[derive(Debug, PartialEq)]
enum State {
    Free,
    Pending,
    Active,
}

pub struct Peer {
    state: State,
    counter: u8,
    incoming_pool: RawFrameSlice,
    mac: u64,
    to_peer: Deque<FrameBox, OUTGOING_SIZE>,
    from_peer: Deque<FrameBox, INCOMING_SIZE>,
}

impl Peer {
    pub const fn const_new() -> Self {
        Self {
            state: State::Free,
            counter: 0,
            incoming_pool: RawFrameSlice::uninit(),
            mac: 0,
            to_peer: Deque::new(),
            from_peer: Deque::new(),
        }
    }

    fn reset_to_free(&mut self) {
        self.to_peer.clear();
        self.from_peer.clear();
        self.mac = 0;
        self.state = State::Free;
        self.counter = 0;
    }

    pub fn promote_to_active(&mut self) {
        if self.state != State::Pending {
            panic!();
        }
        // mac is already set
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Active;
        self.counter = 0;
    }

    pub fn promote_to_pending(&mut self, mac: u64) {
        if self.state != State::Free {
            panic!();
        }
        self.mac = mac;
        self.to_peer.clear();
        self.from_peer.clear();
        self.state = State::Pending;
        self.counter = 0;
    }

    pub fn set_success(&mut self) {
        self.counter = 0;
    }

    pub fn increment_error(&mut self) {
        match self.state {
            State::Free => {
                // uh?
            }
            State::Pending => {
                // one strike, you're out!
                self.reset_to_free();
            }
            State::Active => {
                self.counter += 1;
                if self.counter > 3 {
                    defmt::println!("Resetting active device");
                    self.reset_to_free();
                }
            }
        }
    }

    #[inline]
    pub fn is_pending(&self) -> Option<u64> {
        if self.state == State::Pending {
            Some(self.mac)
        } else {
            None
        }
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.state == State::Active
    }

    #[inline]
    pub fn mac(&self) -> u64 {
        self.mac
    }

    #[inline]
    pub fn is_idle(&mut self) -> bool {
        if self.state != State::Free {
            return false;
        }
        self.incoming_pool.count_allocatable() == INCOMING_SIZE
    }

    pub fn alloc_incoming(&mut self) -> Option<FrameBox> {
        self.incoming_pool.allocate_raw()
    }

    #[inline]
    pub fn is_active_mac(&self, mac: u64) -> bool {
        if self.state != State::Active {
            return false;
        }
        self.mac == mac
    }

    #[inline]
    pub fn enqueue_incoming(&mut self, msg: FrameBox) {
        // The deque length is the same as the pool size,
        // so this should never fail.
        self.from_peer.push_front(msg).map_err(drop).unwrap();
    }

    #[inline]
    pub fn enqueue_outgoing(&mut self, msg: FrameBox) -> Result<(), FrameBox> {
        self.to_peer.push_front(msg)
    }

    #[inline]
    pub fn dequeue_incoming(&mut self) -> Option<FrameBox> {
        self.from_peer.pop_back()
    }

    #[inline]
    pub fn dequeue_outgoing(&mut self) -> Option<FrameBox> {
        self.to_peer.pop_back()
    }

    pub fn new(pool: RawFrameSlice) -> Self {
        Self {
            state: State::Free,
            counter: 0,
            incoming_pool: pool,
            mac: 0,
            to_peer: Deque::new(),
            from_peer: Deque::new(),
        }
    }

    pub fn set_pool(&mut self, pool: RawFrameSlice) {
        self.incoming_pool = pool;
    }
}
