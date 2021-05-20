#![no_std]

mod reg;
pub use crate::reg::*;

#[cfg(feature = "non_blocking")]
mod non_blocking;

#[cfg(feature = "non_blocking")]
pub use non_blocking::*;

#[cfg(not(feature = "non_blocking"))]
mod blocking;

#[cfg(not(feature = "non_blocking"))]
pub use blocking::*;

use core::fmt::Debug;

#[cfg(feature = "out_f32")]
pub use accelerometer::{vector::F32x3, Accelerometer};

#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI communication error
    Spi(SpiError),
    /// CS output pin error
    Pin(PinError),
    InvalidWhoAmI(u8),
}

impl<SpiError, PinError> From<SpiError> for Error<SpiError, PinError> {
    fn from(err: SpiError) -> Self {
        Self::Spi(err)
    }
}

pub struct Lis2dw12<SPI, CS> {
    spi: SPI,
    cs: CS,
    #[cfg(feature = "out_f32")]
    scale: FullScaleSelection,
    #[cfg(feature = "out_f32")]
    #[cfg(not(feature = "non_blocking"))]
    operating_mode: OperatingMode,
}

impl<SPI, CS> Lis2dw12<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
            #[cfg(feature = "out_f32")]
            scale: FullScaleSelection::PlusMinus2G,
            #[cfg(feature = "out_f32")]
            #[cfg(not(feature = "non_blocking"))]
            operating_mode: OperatingMode::LowPower,
        }
    }

    // destroy the instance and return the spi bus and its cs pin
    pub fn destroy(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}
