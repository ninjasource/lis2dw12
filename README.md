# `lis2dw12`

A no_std compatible Rust driver for the low power ST 3-Axis MEMS 14-bit accelerometer LIS2DW12 using the SPI bus

## Example

```rust
// create the device
// where spi implements Transfer<u8, Error = SpiError>,
// and cs implements OutputPin<Error = PinError>
let mut accel = Lis2dw12::new(spi, cs);

// confirm that communication is working
accel.check_who_am_i()?;

// set up the device
accel.set_operating_mode(OperatingMode::HighPerformance)?;
accel.set_low_noise(true)?;
accel.set_full_scale_selection(FullScaleSelection::PlusMinus2G)?;
accel.set_output_data_rate(OutputDataRate::Hp100HzLp100Hz)?; // 100 Hz

// get raw data
let raw = accel.accel_raw()?;
rprintln!("raw: {:?}", raw);

```

or take a look at a working example using an STM32 bluepill [here](https://github.com/ninjasource/accelerometer-test)


> Note that this crate implements the traits in the accelerometer crate. If you have problems figuring out how to propagate errors then take a look at the bluepill working example linked above.

## Features

This crate implements the `RawAccelerometer<I16x3>` trait by default. If you need the `Accelerometer` trait then enable the `out_f32` feature. For example:

```toml
# cargo.toml
lis2dw12 = { version = "0.1.0", features = ["out_f32"] }
```

This crate also exposes an experimental async interface which can be found in the non_blocking module. To use this module you must enable the `non_blocking` feature. This is only available on Rust nightly.

```toml
# cargo.toml for async (requires Nightly)
lis2dw12 = { version = "0.1.0", features = ["out_f32", "non_blocking"] }
```

## Resources

[LIS2DW12 Datasheet](https://www.st.com/resource/en/datasheet/lis2dw12.pdf)

## License

Dual Licensed at your option:

Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)