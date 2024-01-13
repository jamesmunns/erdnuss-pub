//! # Erdnuss Comms
//!
//! This is the "netstack" of the erdnuss project. It's intended to be used on
//! an RS-485 bus. Right now it's only really expected to work on bare metal
//! devices at a fixed network speed of 7.812MHz.
//!
//! This netstack is intended for use on a half-duplex RS-485 bus.
//!
//! At the moment, only 32 devices on a single bus are supported. This also
//! happens to be the upper limit supported by low cost hardware transcievers.
//!
//! At the moment, all communications on the bus are either Controller-to-one-Target, or
//! one-target-to-controller. There is no provision yet for Target-to-Target messaging.
//!
//! ## Entities
//!
//! There are two roles in this netstack:
//!
//! 1. The Controller (or CON) role, which is responsible for commanding and
//!    managing the time slices given to all other devices on the bus
//! 2. The Target (or TGT) role, which only responds to commands from
//!    the Controller.
//!
//! This nomenclature matches those used by I3C and other similar bus protocols
//! with a single device "in charge" of driving communications.
//!
//! At the moment, these roles are expected to be permanently assigned at
//! compile time. There must always be exactly one Controller on any bus.
//!
//! ## Message Framing
//!
//! Messages on the bus are framed by a Line Break, notifying them of an
//! "end of frame" condition. This is intended to allow relatively low active
//! CPU usage, with most nodes in a loop of:
//!
//! 1. Start DMA receive, until a Line Break interrupt occurs
//! 2. Go off and do whatever
//! 3. Once a Line Break occurs, check whether the received message was valid,
//!    and addressed to this node. If yes: process it and potentially respond.
//!    If no: ignore the frame and return to step 1.
//!
//! A line break was chosen because it is well supported by the RP2040 hardware
//! UART implementation.
//!
//! A line break was chosen over 9-bit messages (where the msbit is used as an
//! address/data flag), because the RP2040 doesn't support 9-bit serial, and
//! has no address-match interrupt, which devices like STM32 often have.
//!
//! A line break was chosen over a "line idle" interrupt, because while the RP2040
//! DOES have a line idle interrupt, it does not work when using DMA, as the idle
//! interrupt only fires when the line is quiet AND data is present in the receive
//! FIFO, which will not occur when an RX DMA is actively draining bytes from the
//! receive FIFO.
//!
//! ## Time Division
//!
//! As RS-485 is a half-duplex, shared medium, bus; it is necessary to coordinate
//! betwee all senders to avoid message collisions.
//!
//! This is achieved by having the Controller be "in charge" of a bus. The communication
//! between the Controller and Target is polling-based, and generally looks like this:
//!
//! 1. The line is idle, and the Controller decides to send to a specific Target
//! 2. The Controller sends a command-address byte with the ID of the controller
//! 3. If the Controller has a pending message to that Target, it then sends that payload
//!    (zero or one data frames)
//! 4. Once the Controller is done sending, it signals "end of frame" with a line break, and
//!    begins listening for 1ms, or until a line break occurs, whichever comes first.
//! 5. The addressed Target notices it has been addressed, and all other non-addressed
//!    Targets go back to listening.
//! 6. The addressed Target sends a response-address byte with its own ID
//! 7. If the Target has a pending message for the CON, it then sends that payload
//!    (zero or one data frames)
//! 8. One the Target is done sending, it signals "end of frame" with a line break, and
//!    begins listening again
//! 9. The Controller hears the line break, processes the received message if any, and goes
//!    back to step 1 for the next Target
//!
//! ## Automatic logical addressing
//!
//! All devices are expected to have universally unique 64-bit hardware address, analogous
//! to a MAC-address on ethernet/wifi devices. For RP2040 based nodes, this is typically
//! achieved by using the unique serial number of the QSPI flash chip.
//!
//! In order to reduce overhead on the bus for addressing, devices are dynamically assigned
//! a 5-bit address (0..32).
//!
//! When a Target first boots, it does not have a logical address. The Controller will periodically
//! offer unused addresses, and any Targets without an address will random decide whether to
//! claim the address.
//!
//! As there may be multiple Targets that attempt to claim the address at the same time, the act
//! of being assigned an address takes multiple steps:
//!
//! 1. The Controller offers an address, and includes 8 bytes of random data
//! 2. A Target with no address randomly decides whether to attempt to claim the address.
//!    This random chance is aimed at reducing collisions where two or more nodes attempt
//!    to claim the same address.
//! 3. If a Target decides to go ahead, it takes the 8 random bytes, and XORs them with its own
//!    8-byte unique hardware ID, and sends a "claim" message back
//! 4. If the Controller hears this claim, it takes the received 8 bytes, and XORs them with the
//!    original 8 random bytes. If there was not a collision, it should be left with the MAC
//!    address of the new Target. The Controller marks this address and unique ID as "pending"
//! 5. At a later time, the Controller sends a message to the logical address, containing the MAC address
//!    it thinks it heard in step 4, and waits for an acknowledgement.
//! 6. If the Target hears the logical address it claimed, AND the unique ID matches its own unique ID,
//!    then it sends an acknowledgement, and considers itself as having "joined" the bus, exclusively
//!    owning that logical address
//! 7. If the controller hears the ACK, it marks that address as fully assigned. If it does not hear an
//!    ACK, it marks the address from "pending" to "free".
//!
//! At the moment, the random chance in step 2 is a 1/8 chance, though this may change in the future.
//!
//! ## Controller "steps"
//!
//! So far, we've described the process of a single CON/TGT communication. This must be carried
//! out for all TGTs on the bus. In general, the CON performs an endless polling loop, consisting
//! of three phases:
//!
//! 1. For each known TGT with an assigned logical address:
//!     * Address the TGT, additionally sending it 0 or 1 data frames
//!     * Wait for the TGT to respond.
//!         * If it DOES respond, it will respond with 0 or 1 data frames, and clear the "failure counter".
//!         * If it DOES NOT respond, we increment a "failure counter".
//! 2. For each address in the "pending" phase:
//!     * We send the confirmation message (step 5 above)
//!     * Wait for the TGT to respond or a timeout to occur (step 7 above)
//! 3. If we have any remaining un-assigned logical addresses:
//!     * Send an "offer message" (step 1 above)
//!     * Wait for the TGT to respond or a timeout to occur (step 4 above)
//!
//! At the moment, it is up to the application to decide how often to perform a "step". This could be
//! continuously, every N milliseconds, or on some other metric.
//!
//! More steps/sec means:
//!
//! * More CPU time spent checking and responding to messages
//! * Less latency for messages waiting to be transferred from CON to TGT or TGT to CON
//! * Higher data throughput on the bus
//!
//! Fewer steps/sec will mean the inverse. In the future, there might be a better way to
//! adaptively poll in a more intelligent manner.
//!
//! ## Culling of inactive devices
//!
//! As all Targets are expected to quickly respond to all queries from the Controller, the Controller uses
//! a "three strikes you're out" rule to avoid wasting bus time on timeouts from
//! unresponsive Targets. If a Target fails to respond three times in a row, it is dropped, and
//! the address is marked as free.

#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![allow(async_fn_in_trait)]
#![warn(missing_docs)]

#[macro_use]
mod macros;

pub mod controller;
pub mod frame_pool;
mod peer;
pub mod target;
#[cfg(feature = "postcard-rpc-helpers")]
pub mod wirehelp;
use embassy_time::Instant;

/// The maximum number of Targets supported by a Controller.
pub const MAX_TARGETS: usize = 31;

pub use crate::controller::Controller;

/// An error type for the [`FrameSerial`] trait
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum Error<E> {
    /// Some error with the underlying hardware serial port
    Serial(E),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Serial(value)
    }
}

/// A time-snapshotted data frame
pub struct TimedFrame<'a> {
    /// The timestamp measure as closely as possible to the end
    /// of the frame reception time.
    pub end_of_rx: Instant,
    /// The received data frame
    pub frame: &'a mut [u8],
}

/// A trait representing the communication interface of the RS-485 bus
pub trait FrameSerial {
    /// The error type of the underlying serial port
    type SerError;

    /// Send a single frame.
    ///
    /// The hardware should begin sending the data as soon as possible,
    /// and the future MUST not return until the data is completely done
    /// sending, e.g. all data is "flushed".
    ///
    /// This function is responsible for activating the "send mode" of the
    /// transceiver on entry (e.g. asserting DE), and disabling it on
    /// exit (e.g. deasserting DE).
    ///
    /// This function MUST be cancellation safe, including the de-assertion
    /// of the "send mode".
    async fn send_frame(&mut self, data: &[u8]) -> Result<(), Error<Self::SerError>>;

    /// Receive a single frame, waiting until a Line Break occurs, signalling
    /// the end of a frame
    async fn recv<'a>(
        &mut self,
        frame: &'a mut [u8],
    ) -> Result<TimedFrame<'a>, Error<Self::SerError>>;
}

/// Command + Address byte
///
/// [CmdAddr] is a combined Command and Address byte. It consists of:
///
/// * 3 command-bits (`0..=7`)
/// * 5 address-bits (`0..=31`)
///
/// The command bits are most significant, and the address bits are least
/// significant, e.g. `0bCCC_AAAAA`, where C are command bits and A are
/// address bits.
///
/// The address bits are the logical address of the target, which may be
/// a source or destination, depending on the message kind.
///
/// Commands 1, 2, 4, 5, and 7 are assigned as described below. Commands
/// 0, 3, and 6 are reserved for future use, and currently considered
/// invalid.
#[non_exhaustive]
#[derive(Debug, PartialEq, Eq)]
pub enum CmdAddr {
    /// Select - `0b001`
    ///
    /// Used when the Controller is addressing a Target.
    SelectAddr(u8),
    /// Reply - `0b010`
    ///
    /// Used when the Target is responding to the Controller.
    ReplyFromAddr(u8),
    /// Discovery Offer - `0b100`
    ///
    /// Used when the Controller is offering an unused logical address
    /// to Targets without one. The offered address is the one in the
    /// 5-bit address field.
    DiscoveryOffer(u8),
    /// Discovery Claim - `0b0101`
    ///
    /// Used when a Target attempts to claim a Discovery Offer message.
    DiscoveryClaim(u8),
    /// Discovery Success - `0b111`
    ///
    /// used when the Controller is informing a Target that its address
    /// claim is (tentatively) successful. The Target must respond to
    /// this message with an empty Reply.
    DiscoverySuccess(u8),
}

/// Command Address Error
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum CmdAddrError {
    /// A reserved bit pattern was found
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
