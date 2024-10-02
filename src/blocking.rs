use crate::*;
use accelerometer::{vector::I16x3, RawAccelerometer};
use core::fmt::Debug;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;

#[cfg(feature = "out_f32")]
use num_traits::FromPrimitive;

#[cfg(feature = "out_f32")]
pub use accelerometer::{vector::F32x3, Accelerometer};

impl<SPI, SpiError, CS, PinError> Lis2dw12<SPI, CS>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = PinError>,
{
    pub fn check_who_am_i(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let device_id = self.get_device_id()?;
        if device_id != reg::DEVICE_ID {
            return Err(Error::InvalidWhoAmI(device_id));
        }
        Ok(())
    }

    pub fn set_low_power_mode(
        &mut self,
        low_power_mode: LowPowerMode,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0000_0011;
        self.reg_reset_bits(Register::CTRL1, reset_bits)?;
        self.reg_set_bits(Register::CTRL1, low_power_mode as u8)?;
        Ok(())
    }

    pub fn set_operating_mode(
        &mut self,
        mode: OperatingMode,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0000_1100;
        let set_bits = (mode as u8) << 2;
        self.reg_reset_bits(Register::CTRL1, reset_bits)?;
        self.reg_set_bits(Register::CTRL1, set_bits)?;

        #[cfg(feature = "out_f32")]
        {
            self.operating_mode = mode;
        }

        Ok(())
    }

    pub fn set_low_noise(&mut self, is_enabled: bool) -> Result<(), Error<SpiError, PinError>> {
        let bits = 0b0000_0100;
        if is_enabled {
            self.reg_set_bits(Register::CTRL1, bits)?;
        } else {
            self.reg_reset_bits(Register::CTRL1, bits)?;
        }

        Ok(())
    }

    pub fn set_full_scale_selection(
        &mut self,
        full_scale_selection: FullScaleSelection,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0011_0000;
        let set_bits = (full_scale_selection as u8) << 4;
        self.reg_reset_bits(Register::CTRL6, reset_bits)?;
        self.reg_set_bits(Register::CTRL6, set_bits)?;

        #[cfg(feature = "out_f32")]
        {
            self.scale = full_scale_selection;
        }

        Ok(())
    }

    pub fn set_output_data_rate(
        &mut self,
        odr: OutputDataRate,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b1111_0000;
        let set_bits = (odr as u8) << 4;
        self.reg_reset_bits(Register::CTRL1, reset_bits)?;
        self.reg_set_bits(Register::CTRL1, set_bits)?;
        Ok(())
    }

    pub fn get_device_id(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_reg(Register::WHO_AM_I)
    }

    pub fn get_raw(&mut self) -> Result<I16x3, Error<SpiError, PinError>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_L, &mut buf)?;

        Ok(I16x3::new(
            ((buf[0] as u16) + ((buf[1] as u16) << 8)) as i16,
            ((buf[2] as u16) + ((buf[3] as u16) << 8)) as i16,
            ((buf[4] as u16) + ((buf[5] as u16) << 8)) as i16,
        ))
    }

    /// Get normalized ±g reading from the accelerometer
    #[cfg(feature = "out_f32")]
    pub fn get_norm(&mut self) -> Result<F32x3, Error<SpiError, PinError>> {
        let acc_raw: I16x3 = self.get_raw()?;

        let sensitivity: f32 = match self.scale {
            FullScaleSelection::PlusMinus2G => 0.000061037, // 1 / (MAX(i16) / 2)
            FullScaleSelection::PlusMinus4G => 0.000122074, // 1 / (MAX(i16) / 4)
            FullScaleSelection::PlusMinus8G => 0.000244148, // 1 / (MAX(i16) / 8)
            FullScaleSelection::PlusMinus16G => 0.000488296, // 1 / (MAX(i16) / 16)
        };

        Ok(F32x3::new(
            acc_raw.x as f32 * sensitivity,
            acc_raw.y as f32 * sensitivity,
            acc_raw.z as f32 * sensitivity,
        ))
    }

    fn read_regs(
        &mut self,
        register: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<SpiError, PinError>> {
        // this flag allows us to call read multiple times and the register will automatically be incremented
        const IF_ADD_INC: u8 = 0b0000_0100;
        self.reg_set_bits(Register::CTRL2, IF_ADD_INC)?;

        self.chip_select()?;
        let request = 0b1000_0000 | register.addr(); // set the read bit
        let result = self.write_then_read_into(request, buf);
        self.chip_deselect()?;

        self.reg_reset_bits(Register::CTRL2, IF_ADD_INC)?;
        result
    } // senyo simpson

    fn reg_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<SpiError, PinError>> {
        self.modify_reg(reg, |v| v | bits)
    }

    fn reg_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<SpiError, PinError>> {
        self.modify_reg(reg, |v| v & !bits)
    }

    fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), Error<SpiError, PinError>>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg)?;
        self.write_reg(reg, f(r))?;
        Ok(())
    }

    fn write_reg(&mut self, register: Register, data: u8) -> Result<(), Error<SpiError, PinError>> {
        self.chip_select()?;
        let result = self.write_then_write(register.addr(), data);
        self.chip_deselect()?;
        result
    }

    fn read_reg(&mut self, register: Register) -> Result<u8, Error<SpiError, PinError>> {
        self.chip_select()?;
        let request = 0b1000_0000 | register.addr(); // set the read bit
        let result = self.write_then_read(request);
        self.chip_deselect()?;
        result
    }

    fn write_then_read(&mut self, request: u8) -> Result<u8, Error<SpiError, PinError>> {
        let mut buf = [request];
        self.spi.transfer_in_place(&mut buf)?;
        Ok(buf[0])
    }

    fn write_then_read_into(
        &mut self,
        request: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SpiError, PinError>> {
        self.spi.write(&[request])?;
        Ok(self.spi.read(buf)?)
    }

    fn write_then_write(&mut self, request: u8, data: u8) -> Result<(), Error<SpiError, PinError>> {
        self.spi.write(&[request])?;
        self.spi.write(&[data])?;
        Ok(())
    }

    fn chip_select(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_low().map_err(Error::Pin)
    }

    fn chip_deselect(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_high().map_err(Error::Pin)
    }
}

impl<SPI, SpiError, CS, PinError> RawAccelerometer<I16x3> for Lis2dw12<SPI, CS>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Error = Error<SpiError, PinError>;

    /// Get acceleration reading from the accelerometer
    fn accel_raw(&mut self) -> Result<I16x3, accelerometer::Error<Self::Error>> {
        Ok(self.get_raw()?)
    }
}

#[cfg(feature = "out_f32")]
impl<SPI, SpiError, CS, PinError> Accelerometer for Lis2dw12<SPI, CS>
where
    SPI: SpiDevice< Error = SpiError>,
    CS: OutputPin<Error = PinError>,
    SpiError: Debug,
    PinError: Debug,
{
    type Error = Error<SpiError, PinError>;

    /// Get normalized ±g reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, accelerometer::Error<Self::Error>> {
        let raw = self.get_raw()?;

        let sensitivity: f32 = match self.scale {
            FullScaleSelection::PlusMinus2G => 0.000061037, // 1 / (MAX(i16) / 2)
            FullScaleSelection::PlusMinus4G => 0.000122074, // 1 / (MAX(i16) / 4)
            FullScaleSelection::PlusMinus8G => 0.000244148, // 1 / (MAX(i16) / 8)
            FullScaleSelection::PlusMinus16G => 0.000488296, // 1 / (MAX(i16) / 16)
        };

        Ok(F32x3::new(
            raw.x as f32 * sensitivity,
            raw.y as f32 * sensitivity,
            raw.z as f32 * sensitivity,
        ))
    }

    /// Get sample rate of accelerometer in Hz
    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Self::Error>> {
        let ord_raw = self.read_reg(Register::CTRL1)? >> 4;

        let rate = match self.operating_mode {
            OperatingMode::LowPower => match FromPrimitive::from_u8(ord_raw) {
                Some(OutputDataRate::PowerDown) => 0.0,
                Some(OutputDataRate::Hp12Hz5Lp1Hz6) => 1.6,
                Some(OutputDataRate::Hp12Hz5Lp12Hz5) => 12.5,
                Some(OutputDataRate::Hp25HzLp25Hz) => 25.0,
                Some(OutputDataRate::Hp50HzLp50Hz) => 50.0,
                Some(OutputDataRate::Hp100HzLp100Hz) => 100.0,
                Some(OutputDataRate::Hp200HzLp200Hz) => 200.0,
                Some(OutputDataRate::Hp400HzLp200Hz) => 200.0,
                Some(OutputDataRate::Hp800HzLp200Hz) => 200.0,
                Some(OutputDataRate::Hp1600HzLp200Hz) => 200.0,
                None => 0.0,
            },
            OperatingMode::HighPerformance => match FromPrimitive::from_u8(ord_raw) {
                Some(OutputDataRate::PowerDown) => 0.0,
                Some(OutputDataRate::Hp12Hz5Lp1Hz6) => 12.5,
                Some(OutputDataRate::Hp12Hz5Lp12Hz5) => 12.5,
                Some(OutputDataRate::Hp25HzLp25Hz) => 25.0,
                Some(OutputDataRate::Hp50HzLp50Hz) => 50.0,
                Some(OutputDataRate::Hp100HzLp100Hz) => 100.0,
                Some(OutputDataRate::Hp200HzLp200Hz) => 200.0,
                Some(OutputDataRate::Hp400HzLp200Hz) => 400.0,
                Some(OutputDataRate::Hp800HzLp200Hz) => 800.0,
                Some(OutputDataRate::Hp1600HzLp200Hz) => 1600.0,
                None => 0.0,
            },
            OperatingMode::SingleOnDemand => 0.0,
        };

        Ok(rate)
    }
}
