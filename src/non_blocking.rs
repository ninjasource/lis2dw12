use crate::*;
use accelerometer::vector::I16x3;
use embassy_traits::spi::FullDuplex;
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "out_f32")]
pub use accelerometer::{vector::F32x3, Accelerometer};

impl<SPI, SpiError, CS, PinError> Lis2dw12<SPI, CS>
where
    SPI: FullDuplex<u8, Error = SpiError>,
    CS: OutputPin<Error = PinError>,
{
    pub async fn check_who_am_i(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_high().map_err(Error::Pin)?;
        let device_id = self.get_device_id().await?;
        if device_id != reg::DEVICE_ID {
            return Err(Error::InvalidWhoAmI(device_id));
        }
        Ok(())
    }

    pub async fn set_low_power_mode(
        &mut self,
        low_power_mode: LowPowerMode,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0000_0011;
        self.reg_reset_bits(Register::CTRL1, reset_bits).await?;
        self.reg_set_bits(Register::CTRL1, low_power_mode as u8)
            .await?;
        Ok(())
    }

    pub async fn set_operating_mode(
        &mut self,
        mode: OperatingMode,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0000_1100;
        let set_bits = (mode as u8) << 2;
        self.reg_reset_bits(Register::CTRL1, reset_bits).await?;
        self.reg_set_bits(Register::CTRL1, set_bits).await?;
        Ok(())
    }

    pub async fn set_low_noise(
        &mut self,
        is_enabled: bool,
    ) -> Result<(), Error<SpiError, PinError>> {
        let bits = 0b0000_0100;
        if is_enabled {
            self.reg_set_bits(Register::CTRL1, bits).await?;
        } else {
            self.reg_reset_bits(Register::CTRL1, bits).await?;
        }

        Ok(())
    }

    pub async fn set_full_scale_selection(
        &mut self,
        full_scale_selection: FullScaleSelection,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b0011_0000;
        let set_bits = (full_scale_selection as u8) << 4;
        self.reg_reset_bits(Register::CTRL6, reset_bits).await?;
        self.reg_set_bits(Register::CTRL6, set_bits).await?;

        #[cfg(feature = "out_f32")]
        {
            self.scale = full_scale_selection;
        }

        Ok(())
    }

    pub async fn set_output_data_rate(
        &mut self,
        odr: OutputDataRate,
    ) -> Result<(), Error<SpiError, PinError>> {
        let reset_bits = 0b1111_0000;
        let set_bits = (odr as u8) << 4;
        self.reg_reset_bits(Register::CTRL1, reset_bits).await?;
        self.reg_set_bits(Register::CTRL1, set_bits).await?;
        Ok(())
    }

    pub async fn get_device_id(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_reg(Register::WHO_AM_I).await
    }

    pub async fn get_raw(&mut self) -> Result<I16x3, Error<SpiError, PinError>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_L, &mut buf).await?;

        Ok(I16x3::new(
            ((buf[0] as u16) + ((buf[1] as u16) << 8)) as i16,
            ((buf[2] as u16) + ((buf[3] as u16) << 8)) as i16,
            ((buf[4] as u16) + ((buf[5] as u16) << 8)) as i16,
        ))
    }

    /// Get normalized ±g reading from the accelerometer
    #[cfg(feature = "out_f32")]
    pub async fn get_norm(&mut self) -> Result<F32x3, Error<SpiError, PinError>> {
        let acc_raw: I16x3 = self.get_raw().await?;

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

    async fn read_regs(
        &mut self,
        register: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<SpiError, PinError>> {
        // this flag allows us to call read multiple times and the register will automatically be incremented
        const IF_ADD_INC: u8 = 0b0000_0100;
        self.reg_set_bits(Register::CTRL2, IF_ADD_INC).await?;

        self.chip_select()?;
        let request = 0b1000_0000 | register.addr(); // set the read bit
        let result = self.write_then_read_into(request, buf).await;
        self.chip_deselect()?;

        self.reg_reset_bits(Register::CTRL2, IF_ADD_INC).await?;
        result
    }

    async fn reg_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.modify_reg(reg, |v| v | bits).await
    }

    async fn reg_reset_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.modify_reg(reg, |v| v & !bits).await
    }

    async fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), Error<SpiError, PinError>>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg).await?;
        self.write_reg(reg, f(r)).await?;
        Ok(())
    }

    async fn write_reg(
        &mut self,
        register: Register,
        data: u8,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.chip_select()?;
        let result = self.write_then_write(register.addr(), data).await;
        self.chip_deselect()?;
        result
    }

    async fn read_reg(&mut self, register: Register) -> Result<u8, Error<SpiError, PinError>> {
        self.chip_select()?;
        let request = 0b1000_0000 | register.addr(); // set the read bit
        let result = self.write_then_read(request).await;
        self.chip_deselect()?;
        result
    }

    async fn write_then_read(&mut self, request: u8) -> Result<u8, Error<SpiError, PinError>> {
        self.spi.write(&[request]).await?;
        let mut data = [0; 1];
        self.spi.read(&mut data).await?;
        Ok(data[0])
    }

    async fn write_then_read_into(
        &mut self,
        request: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SpiError, PinError>> {
        self.spi.write(&[request]).await?;

        let mut data = [0; 1];
        for x in buf {
            self.spi.read(&mut data).await?;
            *x = data[0];
        }

        Ok(())
    }

    async fn write_then_write(
        &mut self,
        request: u8,
        data: u8,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.spi.write(&[request]).await?;
        self.spi.write(&[data]).await?;
        Ok(())
    }

    fn chip_select(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_low().map_err(Error::Pin)
    }

    fn chip_deselect(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.cs.set_high().map_err(Error::Pin)
    }
}
