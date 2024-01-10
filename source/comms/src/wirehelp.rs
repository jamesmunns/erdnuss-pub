use crate::frame_pool::FrameBox;
use postcard_rpc::{Endpoint, Topic, WireHeader};
use serde::Serialize;

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
