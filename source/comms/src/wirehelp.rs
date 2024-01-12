use crate::{frame_pool::FrameBox, CmdAddr};
use postcard_rpc::{Endpoint, Topic, WireHeader};
use serde::Serialize;

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

impl SendFrameBox {
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

pub struct WhBody<'a> {
    pub wh: WireHeader,
    pub body: &'a [u8],
}

impl<'a> WhBody<'a> {
    pub fn try_from(fb: &'a FrameBox) -> Option<Self> {
        let (_a, remain) = fb.split_first()?;
        let (wh, body) = postcard_rpc::headered::extract_header_from_bytes(remain).ok()?;
        Some(WhBody { wh, body })
    }
}

#[inline]
fn build_reply_keyed<T: Serialize>(
    mut buf: FrameBox,
    wh: &WireHeader,
    msg: &T,
) -> Option<FrameBox> {
    if buf.is_empty() {
        return None;
    }
    // Split off the address, but DON'T write it!
    //
    // "userspace" doesn't actually know our wire addr, it gets
    // added at send time.
    let (_a, remain) = buf.split_first_mut()?;
    // Then add the wireheader
    let used1 = postcard::to_slice(wh, remain).ok()?.len();
    let (_hdr, remain) = remain.split_at_mut(used1);
    // Then add the body
    let used2 = postcard::to_slice(msg, remain).ok()?.len();

    // TODO: Add CRC?

    let ttl_len = 1 + used1 + used2;
    buf.set_len(ttl_len);

    Some(buf)
}

pub fn send_topic<T>(buf: FrameBox, seq_no: u32, msg: &T::Message) -> Option<FrameBox>
where
    T: Topic,
    T::Message: Serialize,
{
    let wh = WireHeader {
        key: T::TOPIC_KEY,
        seq_no,
    };
    build_reply_keyed::<T::Message>(buf, &wh, msg)
}

pub fn reply_endpoint<E>(buf: FrameBox, seq_no: u32, msg: &E::Response) -> Option<FrameBox>
where
    E: Endpoint,
    E::Response: Serialize,
{
    let wh = WireHeader {
        key: E::RESP_KEY,
        seq_no,
    };
    build_reply_keyed::<E::Response>(buf, &wh, msg)
}
