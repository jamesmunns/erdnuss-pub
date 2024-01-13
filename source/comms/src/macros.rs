#![allow(unused_macros)]

#[cfg(feature = "defmt-logging")]
macro_rules! net_log {
    (trace,   $($arg:expr),*) => { defmt::trace!($($arg),*) };
    (debug,   $($arg:expr),*) => { defmt::debug!($($arg),*) };
    (info,    $($arg:expr),*) => { defmt::info!($($arg),*) };
    (warn,    $($arg:expr),*) => { defmt::warn!($($arg),*) };
    (error,    $($arg:expr),*) => { defmt::error!($($arg),*) };
    (println, $($arg:expr),*) => { defmt::println!($($arg),*) };
}

#[cfg(not(feature = "defmt-logging"))]
macro_rules! nut_log {
    ($level:ident, $($arg:expr),*) => {{ $( let _ = $arg; )* }}
}

macro_rules! nut_trace {
    ($($arg:expr),*) => (nut_log!(trace, $($arg),*));
}

macro_rules! nut_debug {
    ($($arg:expr),*) => (nut_log!(debug, $($arg),*));
}

macro_rules! nut_info {
    ($($arg:expr),*) => (nut_log!(info, $($arg),*));
}

macro_rules! nut_warn {
    ($($arg:expr),*) => (nut_log!(warn, $($arg),*));
}

macro_rules! nut_error {
    ($($arg:expr),*) => (nut_log!(error, $($arg),*));
}

macro_rules! nut_println {
    ($($arg:expr),*) => (nut_log!(println, $($arg),*));
}
