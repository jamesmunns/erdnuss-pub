#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![allow(async_fn_in_trait)]

pub mod dom;
pub mod frame_pool;
pub mod peer;
pub mod sub;
pub mod wirehelp;
use embassy_time::Instant;

pub enum Error<E> {
    Serial(E),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Serial(value)
    }
}

pub struct TimedFrame<'a> {
    pub end_of_rx: Instant,
    pub frame: &'a mut [u8],
}

pub trait FrameSerial {
    type SerError;
    async fn send_frame(&mut self, data: &[u8]) -> Result<(), Error<Self::SerError>>;
    async fn recv<'a>(
        &mut self,
        frame: &'a mut [u8],
    ) -> Result<TimedFrame<'a>, Error<Self::SerError>>;
}

// ```
// 0b000_xxxxx -> reserved (extended addr/cmd?)
//     -> Then `000_AAAAA_BBBBBBBB` where AAAAA is cmd and BBBBBBBB is addr?
// 0b001_NNNNN -> Select address NNNNN (dom tx, then sub rx)
// 0b010_NNNNN -> Reply from address NNNNN
// 0b011_xxxxx -> reserved (broadcast?)
//     -> NoSend Keepalive?
// 0b100_NNNNN -> Discover Offer: NNNNN
// 0b101_NNNNN -> Discovery Claim for NNNNN
// 0b110_xxxxx -> reserved
// 0b111_NNNNN -> Discovery Success for NNNNN
// ```
#[non_exhaustive]
#[derive(Debug, PartialEq, Eq)]
pub enum CmdAddr {
    SelectAddr(u8),
    ReplyFromAddr(u8),
    DiscoveryOffer(u8),
    DiscoveryClaim(u8),
    DiscoverySuccess(u8),
}

#[non_exhaustive]
pub enum CmdAddrError {
    Reserved,
}

impl CmdAddr {
    const SELECT_ADDR: u8 = 0b001;
    const REPLY_FROM_ADDR: u8 = 0b010;
    const DISCOVERY_OFFER: u8 = 0b100;
    const DISCOVERY_CLAIM: u8 = 0b101;
    const DISCOVERY_SUCCESS: u8 = 0b111;
}

impl TryFrom<u8> for CmdAddr {
    type Error = CmdAddrError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let cmd = value >> 5;
        let addr = value & 0b000_11111;
        match cmd {
            Self::SELECT_ADDR => Ok(CmdAddr::SelectAddr(addr)),
            Self::REPLY_FROM_ADDR => Ok(CmdAddr::ReplyFromAddr(addr)),
            Self::DISCOVERY_OFFER => Ok(CmdAddr::DiscoveryOffer(addr)),
            Self::DISCOVERY_CLAIM => Ok(CmdAddr::DiscoveryClaim(addr)),
            Self::DISCOVERY_SUCCESS => Ok(CmdAddr::DiscoverySuccess(addr)),
            _ => Err(CmdAddrError::Reserved),
        }
    }
}

impl From<CmdAddr> for u8 {
    fn from(val: CmdAddr) -> Self {
        let (cmd, addr) = match val {
            CmdAddr::SelectAddr(addr) => (CmdAddr::SELECT_ADDR, addr),
            CmdAddr::ReplyFromAddr(addr) => (CmdAddr::REPLY_FROM_ADDR, addr),
            CmdAddr::DiscoveryOffer(addr) => (CmdAddr::DISCOVERY_OFFER, addr),
            CmdAddr::DiscoveryClaim(addr) => (CmdAddr::DISCOVERY_CLAIM, addr),
            CmdAddr::DiscoverySuccess(addr) => (CmdAddr::DISCOVERY_SUCCESS, addr),
        };
        (cmd << 5) | (addr & 0b000_11111)
    }
}
