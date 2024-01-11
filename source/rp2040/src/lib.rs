#![no_std]

use erdnuss_comms::TimedFrame;
use embassy_rp::{
    flash::{Blocking, Flash},
    gpio::{AnyPin, Output},
    peripherals::FLASH,
    uart::{Async, Instance, Uart},
};
use embassy_time::Instant;
use rand_chacha::{rand_core::SeedableRng, ChaCha8Rng};

pub fn get_unique_id(flash: &mut FLASH) -> Option<u64> {
    let mut flash: Flash<'_, FLASH, Blocking, { 2 * 1024 * 1024 }> = Flash::new_blocking(flash);

    // TODO: For different flash chips, we want to handle things
    // differently based on their jedec? That being said: I control
    // the hardware for this project, and both chips (Pico and XIAO)
    // support unique ID, so oh well.
    //
    // let jedec = flash.blocking_jedec_id().unwrap();

    let mut id = [0u8; core::mem::size_of::<u64>()];
    flash.blocking_unique_id(&mut id).unwrap();
    Some(u64::from_be_bytes(id))
}

pub fn get_rand(unique_id: u64) -> ChaCha8Rng {
    // TODO: Get some real entropy
    let mut seed = [0u8; 32];
    let uid = unique_id.to_le_bytes();
    seed.chunks_exact_mut(8).for_each(|c| {
        c.copy_from_slice(&uid);
    });
    ChaCha8Rng::from_seed(seed)
}

pub struct Rs485Uart<T: Instance + 'static> {
    uart: Uart<'static, T, Async>,
    pin: Output<'static, AnyPin>,
}

impl<T: Instance + 'static> Rs485Uart<T> {
    pub fn new(uart: Uart<'static, T, Async>, mut pin: Output<'static, AnyPin>) -> Self {
        pin.set_low();
        Self { uart, pin }
    }

    /// This function exists so we can do stuff and early return, and still
    /// reset stuff in the outer context.
    //
    // TODO: This function is not cancellation safe wrt the DE pin and RXE! Fix this!
    async fn send_frame_inner(&mut self, data: &[u8]) -> Result<(), erdnuss_comms::Error<()>> {
        // For very short sends, we really don't want to yield, as it will be done
        // almost immediately, and it just adds variability to our timing.
        //
        // At 7.8MHz UART, and 125MHz main CPU, it's 160 CPU cycles/byte,
        // and 10 bytes (1600 CPU cycles) is somewhat arbitrarily picked
        // for two reasons:
        //
        // It's probably not far off the cost of a context switch (about
        // 1000 cycles or so? could be improved running from RAM or actually
        // profiling), and ALSO is large enough to handle a lot of the simple
        // commands, including empty ACKs (one byte), or address ops (usually
        // 9 bytes for header + MAC).
        //
        // This is vibes, not science.
        if data.len() <= 10 {
            self.uart.blocking_write(data).map_err(drop)?;
        } else {
            self.uart.write(data).await.map_err(drop)?;
        }

        // Wait until we're really done. The transmit completes when all bytes
        // are loaded into the FIFO, NOT when the transmitter has drained all
        // bytes. We busy loop until the transmitter is ACTUALLY idle.
        while self.uart.busy() {}

        // Again, we are doing blocking things to remove the variability in async
        // context switching. The embassy code was doing async sleeps, which were
        // ranging from like +0 to +15us of variability.
        //
        // WARNING: This is making assumptions of the UART speed, and won't work
        // for slower speeds! We need to wait ~20 uart clock cycles, or at 7.8MHz,
        // 2.56us. Since we only have single microsecond precision, we wait four
        // microseconds to ensure that we wait long enough.
        //
        // If we lower the clock speed, we SHOULDN'T use 4us as a hardcoded number!
        let start = Instant::now().as_ticks();
        T::regs().uartlcr_h().modify(|w| w.set_brk(true));
        loop {
            // This is doing 64-bit math, which is probably silly.
            // probably worth benchmarking if it's actually a problem
            // before doing any hacks.
            let now = Instant::now().as_ticks();
            if now - start >= 4 {
                break;
            }
        }

        // Line: Broken.
        T::regs().uartlcr_h().modify(|w| w.set_brk(false));

        Ok(())
    }
}

impl<T: Instance + 'static> erdnuss_comms::FrameSerial for Rs485Uart<T> {
    type SerError = ();

    async fn send_frame(&mut self, data: &[u8]) -> Result<(), erdnuss_comms::Error<Self::SerError>> {
        // The XIAO hardware uses a simple resistive divider to scale the 5v RX
        // signal to 3.3v. This works great, EXCEPT that when the RX goes floating
        // when transmitting, the level shifter becomes a pulldown circuit instead,
        // triggering a line break the whole time we're sending. To avoid that,
        // disable the receiver BEFORE we enable transmit mode below
        T::regs().uartcr().modify(|w| w.set_rxe(false));

        // NOTE: We should delay 120ns from enable to actually sending data, this is probably fine
        // with no delays. This would be 8 CPU cycles at 125MHz. Setting up the send is likely to be
        // more than that.
        self.pin.set_high();

        // Send the data, which COULD fail, so don't early return so we can disable
        // the write pin and re-enable the listening mode
        let res = self.send_frame_inner(data).await;

        // Re-enable receive mode on the transceiver, THEN re-enable receive mode
        // on the UART itself.
        self.pin.set_low();
        T::regs().uartcr().modify(|w| w.set_rxe(true));

        res
    }

    async fn recv<'a>(
        &mut self,
        frame: &'a mut [u8],
    ) -> Result<erdnuss_comms::TimedFrame<'a>, erdnuss_comms::Error<Self::SerError>> {
        // This SHOULD already be low.
        self.pin.set_low();

        let ct = self.uart.read_to_break(frame).await.map_err(drop)?;
        // TODO: It would be nice in the future to grab this instant in the
        // interrupt somehow, for better accuracy.
        let now = Instant::now();
        Ok(TimedFrame {
            end_of_rx: now,
            frame: &mut frame[..ct],
        })
    }
}
