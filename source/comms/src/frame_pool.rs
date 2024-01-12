use core::{
    ops::{Deref, DerefMut},
    ptr::{addr_of, addr_of_mut, NonNull},
    sync::atomic::{AtomicU8, Ordering},
    unreachable,
};
use grounded::{const_init::ConstInit, uninit::GroundedArrayCell};

#[repr(C)]
pub struct RawFrame {
    data: [u8; 255],
    freelen: AtomicU8,
}

pub struct FrameBox {
    ptr: NonNull<RawFrame>,
}

unsafe impl Send for FrameBox {}

impl FrameBox {
    unsafe fn freelen_ref(&self) -> &AtomicU8 {
        let fl_ptr = addr_of!((*self.ptr.as_ptr()).freelen);
        &*fl_ptr
    }

    pub fn set_len(&mut self, len: usize) {
        if len == 0 || len > 255 {
            unreachable!()
        }
        unsafe {
            let fl = self.freelen_ref();
            fl.store(len as u8, Ordering::Relaxed);
        }
    }
}

impl Deref for FrameBox {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        let len = unsafe { self.freelen_ref().load(Ordering::Relaxed) };
        assert!(len != 0);
        let data_ptr: *const u8 = unsafe {
            let arr_ptr: *const [u8; 255] = addr_of!((*self.ptr.as_ptr()).data);
            arr_ptr.cast()
        };
        unsafe { core::slice::from_raw_parts(data_ptr, len as usize) }
    }
}

impl DerefMut for FrameBox {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let len = unsafe { self.freelen_ref().load(Ordering::Relaxed) };
        assert!(len != 0);
        let data_ptr: *mut u8 = unsafe {
            let arr_ptr: *mut [u8; 255] = addr_of_mut!((*self.ptr.as_ptr()).data);
            arr_ptr.cast()
        };
        unsafe { core::slice::from_raw_parts_mut(data_ptr, len as usize) }
    }
}

impl Drop for FrameBox {
    fn drop(&mut self) {
        let ptr: *mut RawFrame = self.ptr.as_ptr();
        // SAFETY: FrameBox represents ownership of `data`, and we have the right
        // to release on drop
        unsafe {
            let atom_ptr: *mut AtomicU8 = addr_of_mut!((*ptr).freelen);
            let atom: &AtomicU8 = &*atom_ptr;
            atom.store(RawFrame::FREE, Ordering::Release);
        }
    }
}

impl RawFrame {
    const FREE: u8 = 0;
    const MAX_LEN: u8 = 255;
}

impl ConstInit for RawFrame {
    #[allow(clippy::declare_interior_mutable_const)]
    const VAL: Self = RawFrame {
        data: [0u8; Self::MAX_LEN as usize],
        freelen: AtomicU8::new(0),
    };
}

unsafe impl Send for RawFrameSlice {}

pub struct RawFrameSlice {
    start: NonNull<RawFrame>,
    len: usize,
    next_idx: usize,
}

impl RawFrameSlice {
    /// ## Safety
    ///
    /// You must only ever call this once
    pub unsafe fn from_static<const N: usize>(
        buf: &'static GroundedArrayCell<RawFrame, N>,
    ) -> Self {
        Self {
            start: NonNull::new_unchecked(buf.as_mut_ptr()),
            len: N,
            next_idx: 0,
        }
    }

    pub const fn uninit() -> Self {
        Self {
            start: NonNull::dangling(),
            len: 0,
            next_idx: 0,
        }
    }

    pub fn count_allocatable(&self) -> usize {
        if self.len == 0 {
            return 0;
        }
        let mut ct = 0;
        let start_ptr: *mut RawFrame = self.start.as_ptr();
        for idx in 0..self.len {
            let ptr: *mut RawFrame = unsafe { start_ptr.add(idx) };
            // Scope access of subfield so all references are dropped before we make the
            // non-null
            {
                let atom_ptr: *const AtomicU8 = unsafe { addr_of!((*ptr).freelen) };
                let atom: &AtomicU8 = unsafe { &*atom_ptr };

                // IF the value is zero, we have mutable exclusive access to allocate it.
                if atom.load(Ordering::Acquire) == RawFrame::FREE {
                    ct += 1;
                }
            }
        }
        ct
    }

    pub fn allocate_raw(&mut self) -> Option<FrameBox> {
        if self.len == 0 {
            return None;
        }
        if self.next_idx >= self.len {
            self.next_idx = 0;
        }
        let start_ptr: *mut RawFrame = self.start.as_ptr();
        let idxes = (self.next_idx..self.len).chain(0..self.next_idx);
        for idx in idxes {
            let ptr: *mut RawFrame = unsafe { start_ptr.add(idx) };
            // Scope access of subfield so all references are dropped before we make the
            // non-null
            {
                let atom_ptr: *mut AtomicU8 = unsafe { addr_of_mut!((*ptr).freelen) };
                let atom: &AtomicU8 = unsafe { &*atom_ptr };

                // IF the value is zero, we have mutable exclusive access to allocate it.
                if atom.load(Ordering::Acquire) == RawFrame::FREE {
                    atom.store(RawFrame::MAX_LEN, Ordering::Release);
                    self.next_idx = idx + 1;
                } else {
                    continue;
                }
            }
            // If we didn't continue, we succeeded, and the len is now MAX_LEN
            return Some(FrameBox {
                ptr: NonNull::new(ptr)?,
            });
        }

        // End of search, none found
        None
    }

    /// Splits the tail starting at `at` from self.
    ///
    /// Additionally will refuse to split if `at` is `0` or the current capacity.
    ///
    /// Self is left with elements `[..at]`, and the new item is left with elements `[at..]`.
    pub fn split(&mut self, at: usize) -> Option<Self> {
        if (at == 0) || (at > self.len) {
            return None;
        }

        // if len = 5, and at = 2, then:
        // self.len becomes 2 (0, 1)
        // new.len becomes 3 (2, 3, 4)
        let len_new = self.len - at;
        self.len = at;

        Some(RawFrameSlice {
            start: unsafe { NonNull::new_unchecked(self.start.as_ptr().add(at)) },
            len: len_new,
            next_idx: 0,
        })
    }

    pub fn capacity(&self) -> usize {
        self.len
    }
}
