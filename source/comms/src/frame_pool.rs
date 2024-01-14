//! Frame Pool
//!
//! This is a specialized pool allocator. It is optimized and
//! opinionated for the following cases:
//!
//! * Use where you want to store multiple chunks of bytes,
//!   in the size range of 1..=255 bytes
//! * Use in cases where the target may not have CAS atomics,
//!   so only `load` and `stores` are used for synchronization
//!
//! This allows for the creation of [`FrameBox`] allocations, that
//! can only be allocated with exclusive access to a [`RawFrameSlice`],
//! but can be deallocated by dropping (just like a Box from the standard
//! library), and do not require any kind of mutex at the time of drop.

use core::{
    ops::{Deref, DerefMut},
    ptr::{addr_of, addr_of_mut, NonNull},
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
    unreachable,
};
use grounded::{const_init::ConstInit, uninit::GroundedArrayCell};

use crate::CmdAddr;

/// Storage for exactly N frames
pub struct FrameStorage<const N: usize> {
    frames: GroundedArrayCell<RawFrame, N>,
    once: AtomicBool,
}

impl<const N: usize> FrameStorage<N> {
    /// Create a new frame storage buffer
    ///
    /// Intended for static usage.
    pub const fn new() -> Self {
        Self {
            frames: GroundedArrayCell::const_init(),
            once: AtomicBool::new(false),
        }
    }

    /// Attempt to take the storage as a [RawFrameSlice]
    ///
    /// The first call will return Some, all later calls will
    /// return None. Uses a [critical section][critical_section::with]
    /// to ensure it only works once, even on targets without atomics
    pub fn take(&'static self) -> Option<RawFrameSlice> {
        self.take_gac()
            .map(|s| unsafe { RawFrameSlice::from_static(s) })
    }

    fn take_gac(&'static self) -> Option<&'static GroundedArrayCell<RawFrame, N>> {
        critical_section::with(|_| {
            let old = self.once.load(Ordering::Acquire);
            self.once.store(true, Ordering::Release);
            !old
        })
        .then_some(&self.frames)
    }
}

/// The Rules:
///
/// `freelen` serves two functions:
///
/// * When NOT allocated, it must always be zero.
/// * When allocated, it represents the "len" of the frame, and
///   must ALWAYS be NONZERO.
///
/// ONLY the RawFrameSlice is allowed to make the zero -> nonzero
/// transition, when the freelen is nonzero, it MUST NOT read or write
/// the data field, nor write to freelen.
///
/// ONLY the FrameBox is allowed to make the nonzero -> zero transition.
/// Setting freelen to zero represents giving up exclusive access to the
/// contents of the data field.
#[repr(C)]
pub(crate) struct RawFrame {
    data: [u8; 255],
    freelen: AtomicU8,
}

/// An allocated frame storage
///
/// Stores `1..=255` bytes. Storage can be accessed through the
/// [Deref] and [DerefMut] traits.
pub struct FrameBox {
    ptr: NonNull<RawFrame>,
}

unsafe impl Send for FrameBox {}

impl FrameBox {
    unsafe fn freelen_ref(&self) -> &AtomicU8 {
        let fl_ptr = addr_of!((*self.ptr.as_ptr()).freelen);
        &*fl_ptr
    }

    /// Sets the length of the storage.
    ///
    /// ## Panics
    ///
    /// `len` must be >= 1 and <= 255 or this function will panic
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
        // Ordering can be relaxed as we have exclusive access to the
        // backing storage, and exclusive WRITE access to freelen as
        // long as the FrameBox exists.
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
        // Ordering can be relaxed as we have exclusive access to the
        // backing storage, and exclusive WRITE access to freelen as
        // long as the FrameBox exists.
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

/// A sliceable allocation pool
///
/// Can be created via [FrameStorage::take()], or by splitting
/// via [RawFrameSlice::split()].
pub struct RawFrameSlice {
    start: NonNull<RawFrame>,
    len: usize,
    next_idx: usize,
}

impl RawFrameSlice {
    /// ## Safety
    ///
    /// You must only ever call this once
    pub(crate) unsafe fn from_static<const N: usize>(
        buf: &'static GroundedArrayCell<RawFrame, N>,
    ) -> Self {
        Self {
            start: NonNull::new_unchecked(buf.as_mut_ptr()),
            len: N,
            next_idx: 0,
        }
    }

    /// Create a new, empty [RawFrameSlice] that has no
    /// backing storage.
    pub const fn uninit() -> Self {
        Self {
            start: NonNull::dangling(),
            len: 0,
            next_idx: 0,
        }
    }

    /// Count the number of allocatable items
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

    /// Attempt to allocate a [FrameBox] from the backing storage
    /// available to this [RawFrameSlice].
    ///
    /// This allocation performs a linear search of the backing
    /// storage, so allocation is `O(n)`. Returns [None] if no
    /// storage slots were available.
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

    /// The backing capacity of this [RawFrameSlice].
    pub fn capacity(&self) -> usize {
        self.len
    }
}

/// WireFrameBox represents a valid packet received from the wire
///
/// It is guaranteed to have a valid `CmdAddr`. It may have no data if
/// it is an empty message
pub struct WireFrameBox {
    fb: FrameBox,
}

impl WireFrameBox {
    pub(crate) fn new_unchecked(fb: FrameBox) -> Self {
        Self { fb }
    }

    /// Get the [CmdAddr] of this message
    #[inline]
    pub fn cmd_addr(&self) -> CmdAddr {
        self.fb[0].try_into().unwrap()
    }

    /// Borrow the payload
    #[inline]
    pub fn payload(&self) -> &[u8] {
        &self.fb[1..]
    }

    /// Mutably borrow the payload
    #[inline]
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.fb[1..]
    }

    /// Deconstruct the WireFrameBox
    #[inline]
    pub fn into_inner(self) -> FrameBox {
        self.fb
    }

    /// Is the PAYLOAD empty?
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.fb.len() == 1
    }

    /// The PAYLOAD len
    #[inline]
    pub fn len(&self) -> usize {
        self.fb.len() - 1
    }

    /// Set the length of the frame, NOT counting the [CmdAddr] field
    pub fn set_len(&mut self, len: usize) {
        self.fb.set_len(1 + len)
    }
}

/// SendFrameBox represent a message to be sent over the wire.
///
/// Unlike [WireFrameBox], it does NOT have a valid [CmdAddr], which is
/// assigned at sending time.
pub struct SendFrameBox {
    fb: FrameBox,
}

impl From<FrameBox> for SendFrameBox {
    fn from(value: FrameBox) -> Self {
        Self { fb: value }
    }
}

impl SendFrameBox {
    /// Borrow the payload
    #[inline]
    pub fn payload(&self) -> &[u8] {
        &self.fb[1..]
    }

    /// Mutably borrow the payload
    #[inline]
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.fb[1..]
    }

    /// Deconstruct the SendFrameBox
    #[inline]
    pub fn into_inner(self) -> FrameBox {
        self.fb
    }

    /// Is the PAYLOAD empty?
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.fb.len() == 1
    }

    /// The PAYLOAD len
    #[inline]
    pub fn len(&self) -> usize {
        self.fb.len() - 1
    }

    /// Set the length of the frame, NOT counting the [CmdAddr] field
    pub fn set_len(&mut self, len: usize) {
        self.fb.set_len(1 + len)
    }
}
