# `lis2dw12`

A no_std compatible Rust driver for the low power ST 3-Axis MEMS accelerometer LIS2DW12 using the SPI bus

## Example

```rust
// create the device
// where spi implements Transfer<u8, Error = SpiError>,
// and cs implements OutputPin<Error = PinError>
let mut accel = Lis2dw12::new(spi, cs);

// confirm that communication is working
accel.check_who_am_i()?;

// set up the device
accel.set_mode(OperatingMode::HighPerformance)?;
accel.set_low_noise(true)?;
accel.set_full_scale_selection(FullScaleSelection::PlusMinus2)?;
accel.set_output_data_rate(OutputDataRate::Hp100HzLp100Hz)?; // 100 Hz

// get raw data
let raw = accel.accel_raw()?;
rprintln!("raw: {:?}", raw);

```

or take a look at a working example using an STM32 bluepill [here](https://github.com/ninjasource/accelerometer-test)


> Note that this crate implements the traits in the accelerometer crate. If you have problems figuring out how to propogate errors then take a look at the working example


## License

Dual Licensed at your option:

Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)