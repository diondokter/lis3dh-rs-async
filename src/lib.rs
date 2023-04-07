//! Platform-agnostic LIS3DH accelerometer driver which uses I²C via
//! [embedded-hal]. This driver implements the [`Accelerometer`][acc-trait]
//! and [`RawAccelerometer`][raw-trait] traits from the [accelerometer] crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [accelerometer]: https://docs.rs/accelerometer
//! [acc-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//! [raw-trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.RawAccelerometer.html
//!

#![no_std]
#![feature(type_alias_impl_trait)]

use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use core::future::Future;

use accelerometer::vector::{F32x3, I16x3};
use embedded_hal::spi::SpiBusWrite;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::{SpiBus, SpiDevice};

mod interrupts;
mod register;

use interrupts::*;
pub use interrupts::{
    Detect4D, Interrupt1, Interrupt2, InterruptConfig, InterruptMode, InterruptSource, IrqPin,
    IrqPin1Config, IrqPin2Config, LatchInterruptRequest,
};

use register::*;
pub use register::{
    DataRate, DataStatus, Duration, FifoMode, FifoStatus, Mode, Range, Register, SlaveAddr,
    Threshold,
};

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid operating mode selection
    InvalidMode,

    /// Invalid full-scale selection
    InvalidRange,

    /// Attempted to write to a read-only register
    WriteToReadOnly,

    /// Invalid address provided
    WrongAddress,
}

/// `LIS3DH` driver.
pub struct Lis3dh<CORE> {
    core: CORE,
}

impl<I2C, E> Lis3dh<Lis3dhI2C<I2C>>
where
    I2C: I2c<Error = E>,
{
    /// Create a new LIS3DH driver from the given I2C peripheral using the default config.
    /// Default is Hz_400 HighResolution.
    /// An example using the [nrf52840_hal](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/index.html):
    ///
    /// ```rust,ignore
    /// use nrf52840_hal::gpio::{Level, PushPull};
    /// use lis3dh::Lis3dh;
    ///
    /// let peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();
    /// let pins = p0::Parts::new(peripherals.P0);
    ///
    /// let twim0_scl = pins.p0_31.into_floating_input().degrade();
    /// let twim0_sda = pins.p0_30.into_floating_input().degrade();
    ///
    /// let i2c = nrf52840_hal::twim::Twim::new(
    ///     peripherals.TWIM0,
    ///     nrf52840_hal::twim::Pins {
    ///         scl: twim0_scl,
    ///         sda: twim0_sda,
    ///     },
    ///     nrf52840_hal::twim::Frequency::K400,
    /// );
    ///
    /// let lis3dh = Lis3dh::new_i2c(i2c, lis3dh::SlaveAddr::Default).unwrap();
    /// ```
    pub async fn new_i2c(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E>> {
        Self::new_i2c_with_config(i2c, address, Configuration::default()).await
    }

    /// Create a new driver instance without talking to the LIS3DH.
    /// This can be useful when the accelerometer was already on while the microcontroller rebooted and you need
    /// continuous operation.
    pub async fn new_i2c_without_config(i2c: I2C, address: SlaveAddr) -> Self {
        let core = Lis3dhI2C {
            i2c,
            address: address.addr(),
        };

        Lis3dh { core }
    }

    pub async fn new_i2c_with_config(
        i2c: I2C,
        address: SlaveAddr,
        config: Configuration,
    ) -> Result<Self, Error<E>> {
        let core = Lis3dhI2C {
            i2c,
            address: address.addr(),
        };

        let mut lis3dh = Lis3dh { core };

        lis3dh.configure(config).await?;

        Ok(lis3dh)
    }
}

impl<SPI, ESPI> Lis3dh<Lis3dhSPI<SPI>>
where
    SPI: SpiDevice<Error = ESPI>,
{
    /// Create a new LIS3DH driver from the given SPI peripheral.
    /// An example using the [nrf52840_hal](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/index.html):
    ///
    /// ```rust,ignore
    /// use nrf52840_hal::gpio::{p0::{Parts, P0_28}, *};
    /// use nrf52840_hal::spim::Spim;
    /// use lis3dh::Lis3dh;
    ///
    /// let peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();
    /// let port0 = Parts::new(peripherals.P0);
    ///
    /// // define the chip select pin
    /// let cs: P0_28<Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::High);
    ///
    /// // spi pins: clock, miso, mosi
    /// let pins = nrf52840_hal::spim::Pins {
    ///     sck: port0.p0_31.into_push_pull_output(Level::Low).degrade(),
    ///     miso: Some(port0.p0_30.into_push_pull_output(Level::Low).degrade()),
    ///     mosi: Some(port0.p0_29.into_floating_input().degrade()),
    /// };
    ///
    /// // set up the spi peripheral
    /// let spi = Spim::new(
    ///     peripherals.SPIM2,
    ///     pins,
    ///     nrf52840_hal::spim::Frequency::K500,
    ///     nrf52840_hal::spim::MODE_0,
    ///     0,
    /// );
    /// // create and initialize the sensor
    /// let lis3dh = Lis3dh::new_spi(spi, cs).unwrap();
    /// ```
    pub async fn new_spi(spi: SPI) -> Result<Self, Error<ESPI>> {
        Self::new_spi_with_config(spi, Configuration::default()).await
    }

    /// Create a new driver instance without talking to the LIS3DH.
    /// This can be useful when the accelerometer was already on while the microcontroller rebooted and you need
    /// continuous operation.
    pub async fn new_spi_without_config(spi: SPI) -> Self {
        let core = Lis3dhSPI { spi };

        Lis3dh { core }
    }

    pub async fn new_spi_with_config(spi: SPI, config: Configuration) -> Result<Self, Error<ESPI>> {
        let core = Lis3dhSPI { spi };

        let mut lis3dh = Lis3dh { core };

        lis3dh.configure(config).await?;

        Ok(lis3dh)
    }
}

impl<CORE> Lis3dh<CORE>
where
    CORE: Lis3dhCore,
{
    /// Configure the device
    pub async fn configure(&mut self, conf: Configuration) -> Result<(), Error<CORE::BusError>> {
        if self.get_device_id().await? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        if conf.block_data_update || conf.enable_temperature {
            // Block data update
            self.write_register(Register::CTRL4, BDU).await?;
        }

        self.set_mode(conf.mode).await?;

        self.set_datarate(conf.datarate).await?;

        self.enable_axis((conf.enable_x_axis, conf.enable_y_axis, conf.enable_z_axis))
            .await?;

        if conf.enable_temperature {
            self.enable_temp(true).await?;
        }

        // Enable ADCs.
        self.write_register(Register::TEMP_CFG, ADC_EN).await
    }

    /// `WHO_AM_I` register.
    pub async fn get_device_id(&mut self) -> Result<u8, Error<CORE::BusError>> {
        self.read_register(Register::WHOAMI).await
    }

    /// X,Y,Z-axis enable.
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    async fn enable_axis(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<CORE::BusError>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !(X_EN | Y_EN | Z_EN); // disable all axes

            ctrl1 |= if x { X_EN } else { 0 };
            ctrl1 |= if y { Y_EN } else { 0 };
            ctrl1 |= if z { Z_EN } else { 0 };

            ctrl1
        })
        .await
    }

    /// Operating mode selection.
    /// `CTRL_REG1`: `LPen` bit, `CTRL_REG4`: `HR` bit.
    /// You need to wait for stabilization after setting. In future this
    /// function will be deprecated and instead take a `Delay` to do this for
    /// you.
    ///
    /// | From           | To             | Wait for   |
    /// |:---------------|:---------------|:-----------|
    /// | HighResolution | LowPower       | 1/datarate |
    /// | HighResolution | Normal         | 1/datarate |
    /// | Normal         | LowPower       | 1/datarate |
    /// | Normal         | HighResolution | 7/datarate |
    /// | LowPower       | Normal         | 1/datarate |
    /// | LowPower       | HighResolution | 7/datarate |
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<CORE::BusError>> {
        match mode {
            Mode::LowPower => {
                self.register_set_bits(Register::CTRL1, LP_EN).await?;
                self.register_clear_bits(Register::CTRL4, HR).await?;
            }
            Mode::Normal => {
                self.register_clear_bits(Register::CTRL1, LP_EN).await?;
                self.register_clear_bits(Register::CTRL4, HR).await?;
            }
            Mode::HighResolution => {
                self.register_clear_bits(Register::CTRL1, LP_EN).await?;
                self.register_set_bits(Register::CTRL4, HR).await?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode.
    pub async fn get_mode(&mut self) -> Result<Mode, Error<CORE::BusError>> {
        let ctrl1 = self.read_register(Register::CTRL1).await?;
        let ctrl4 = self.read_register(Register::CTRL4).await?;

        let is_lp_set = (ctrl1 >> 3) & 0x01 != 0;
        let is_hr_set = (ctrl4 >> 3) & 0x01 != 0;

        let mode = match (is_lp_set, is_hr_set) {
            (true, false) => Mode::LowPower,
            (false, false) => Mode::Normal,
            (false, true) => Mode::HighResolution,
            _ => return Err(Error::InvalidMode),
        };

        Ok(mode)
    }

    /// Data rate selection.
    pub async fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<CORE::BusError>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            // Mask off lowest 4 bits
            ctrl1 &= !ODR_MASK;
            // Write in new output data rate to highest 4 bits
            ctrl1 |= datarate.bits() << 4;

            ctrl1
        })
        .await
    }

    /// Read the current data selection rate.
    pub async fn get_datarate(&mut self) -> Result<DataRate, Error<CORE::BusError>> {
        let ctrl1 = self.read_register(Register::CTRL1).await?;
        let odr = (ctrl1 >> 4) & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Full-scale selection.
    pub async fn set_range(&mut self, range: Range) -> Result<(), Error<CORE::BusError>> {
        self.modify_register(Register::CTRL4, |mut ctrl4| {
            // Mask off lowest 4 bits
            ctrl4 &= !FS_MASK;
            // Write in new full-scale to highest 4 bits
            ctrl4 |= range.bits() << 4;

            ctrl4
        })
        .await
    }

    /// Read the current full-scale.
    pub async fn get_range(&mut self) -> Result<Range, Error<CORE::BusError>> {
        let ctrl4 = self.read_register(Register::CTRL4).await?;
        let fs = (ctrl4 >> 4) & 0b0011;

        Range::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Set `REFERENCE` register.
    pub async fn set_ref(&mut self, reference: u8) -> Result<(), Error<CORE::BusError>> {
        self.write_register(Register::REFERENCE, reference).await
    }

    /// Read the `REFERENCE` register.
    pub async fn get_ref(&mut self) -> Result<u8, Error<CORE::BusError>> {
        self.read_register(Register::REFERENCE).await
    }

    /// Accelerometer data-available status.
    pub async fn get_status(&mut self) -> Result<DataStatus, Error<CORE::BusError>> {
        let stat = self.read_register(Register::STATUS).await?;

        Ok(DataStatus {
            zyxor: (stat & ZYXOR) != 0,
            xyzor: ((stat & XOR) != 0, (stat & YOR) != 0, (stat & ZOR) != 0),
            zyxda: (stat & ZYXDA) != 0,
            xyzda: ((stat & XDA) != 0, (stat & YDA) != 0, (stat & ZDA) != 0),
        })
    }

    /// Convenience function for `STATUS_REG` to confirm all three X, Y and
    /// Z-axis have new data available for reading by accel_raw and associated
    /// function calls.
    pub async fn is_data_ready(&mut self) -> Result<bool, Error<CORE::BusError>> {
        let value = self.get_status().await?;

        Ok(value.zyxda)
    }

    /// Temperature sensor enable.
    /// `TEMP_CGF_REG`: `TEMP_EN`, the BDU bit in `CTRL_REG4` is also set.
    pub async fn enable_temp(&mut self, enable: bool) -> Result<(), Error<CORE::BusError>> {
        self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)
            .await?;

        // enable block data update (required for temp reading)
        if enable {
            self.register_xset_bits(Register::CTRL4, BDU, true).await?;
        }

        Ok(())
    }

    /// Raw temperature sensor data as `i16`. The temperature sensor __must__
    /// be enabled via `enable_temp` prior to reading.
    pub async fn get_temp_out(&mut self) -> Result<i16, Error<CORE::BusError>> {
        let out_l = self.read_register(Register::OUT_ADC3_L).await?;
        let out_h = self.read_register(Register::OUT_ADC3_H).await?;

        Ok(i16::from_le_bytes([out_l, out_h]))
    }

    /// Temperature sensor data converted to `f32`. Output is in degree
    /// celsius. The temperature sensor __must__ be enabled via `enable_temp`
    /// prior to reading.
    pub async fn get_temp_outf(&mut self) -> Result<f32, Error<CORE::BusError>> {
        let temp_out = self.get_temp_out().await?;

        Ok(temp_out as f32 / 256.0 + 25.0)
    }

    /// Modify a register's value. Read the current value of the register,
    /// update the value with the provided function, and set the register to
    /// the return value.
    async fn modify_register<F>(
        &mut self,
        register: Register,
        f: F,
    ) -> Result<(), Error<CORE::BusError>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register).await?;

        self.write_register(register, f(value)).await
    }

    /// Clear the given bits in the given register. For example:
    ///
    /// ```rust,ignore
    /// lis3dh.register_clear_bits(0b0110)
    /// ```
    /// This call clears (sets to 0) the bits at index 1 and 2. Other bits of the register are not touched.
    pub async fn register_clear_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<CORE::BusError>> {
        self.modify_register(reg, |v| v & !bits).await
    }

    /// Set the given bits in the given register. For example:
    ///
    /// ```rust,ignore
    /// lis3dh.register_set_bits(0b0110)
    /// ```
    ///
    /// This call sets to 1 the bits at index 1 and 2. Other bits of the register are not touched.
    pub async fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<CORE::BusError>> {
        self.modify_register(reg, |v| v | bits).await
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    async fn register_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), Error<CORE::BusError>> {
        if set {
            self.register_set_bits(reg, bits).await
        } else {
            self.register_clear_bits(reg, bits).await
        }
    }

    /// Configure one of the interrupt pins
    ///
    /// ```rust,ignore
    /// lis3dh.configure_interrupt_pin(IrqPin1Config {
    ///     // Raise if interrupt 1 is raised
    ///     ia1_en: true,
    ///     // Disable for all other interrupts
    ///     ..IrqPin1Config::default()
    /// })?;
    /// ```
    pub async fn configure_interrupt_pin<P: IrqPin>(
        &mut self,
        pin: P,
    ) -> Result<(), Error<CORE::BusError>> {
        self.write_register(P::ctrl_reg(), pin.bits()).await
    }

    /// Configure an IRQ source
    ///
    /// Example: configure interrupt 1 to fire when there is movement along any of the axes.
    ///
    /// ```rust,ignore
    /// lis3dh.configure_irq_src(
    ///     lis3dh::Interrupt1,
    ///     lis3dh::InterruptMode::Movement,
    ///     lis3dh::InterruptConfig::high_and_low(),
    /// )?;
    /// ```
    pub async fn configure_irq_src<I: Interrupt>(
        &mut self,
        int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
    ) -> Result<(), Error<CORE::BusError>> {
        self.configure_irq_src_and_control(
            int,
            interrupt_mode,
            interrupt_config,
            LatchInterruptRequest::default(),
            Detect4D::default(),
        )
        .await
    }

    /// Configure an IRQ source.
    ///
    /// LIS (latch interrupt request) will latch (keep active) the interrupt until the [`Lis3dh::get_irq_src`] is read.
    ///
    /// 4D detection is a subset of the 6D detection where detection on the Z axis is disabled.
    /// This setting only has effect when the interrupt mode is either `Movement` or `Position`.
    ///
    /// Example: configure interrupt 1 to fire when there is movement along any of the axes.
    ///
    /// ```rust,ignore
    /// lis3dh.configure_irq_src(
    ///     lis3dh::Interrupt1,
    ///     lis3dh::InterruptMode::Movement,
    ///     lis3dh::InterruptConfig::high_and_low(),
    ///     lis3dh::LatchInterruptRequest::Enable,
    ///     lis3dh::Detect4D::Enable,
    /// )?;
    /// ```
    pub async fn configure_irq_src_and_control<I: Interrupt>(
        &mut self,
        _int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
        latch_interrupt_request: LatchInterruptRequest,
        detect_4d: Detect4D,
    ) -> Result<(), Error<CORE::BusError>> {
        let latch_interrupt_request =
            matches!(latch_interrupt_request, LatchInterruptRequest::Enable);

        let detect_4d = matches!(detect_4d, Detect4D::Enable);

        if latch_interrupt_request || detect_4d {
            let latch = (latch_interrupt_request as u8) << I::lir_int_bit();
            let d4d = (detect_4d as u8) << I::d4d_int_bit();
            self.register_set_bits(Register::CTRL5, latch | d4d).await?;
        }
        self.write_register(I::cfg_reg(), interrupt_config.to_bits(interrupt_mode))
            .await
    }

    /// Set the minimum duration for the Interrupt event to be recognized.
    ///
    /// Example: the event has to last at least 25 miliseconds to be recognized.
    ///
    /// ```rust,ignore
    /// // let mut lis3dh = ...
    /// let duration = Duration::miliseconds(DataRate::Hz_400, 25.0);
    /// lis3dh.configure_irq_duration(duration);
    /// ```
    #[doc(alias = "INT1_DURATION")]
    #[doc(alias = "INT2_DURATION")]
    pub async fn configure_irq_duration<I: Interrupt>(
        &mut self,
        _int: I,
        duration: Duration,
    ) -> Result<(), Error<CORE::BusError>> {
        self.write_register(I::duration_reg(), duration.0).await
    }

    /// Set the minimum magnitude for the Interrupt event to be recognized.
    ///
    /// Example: the event has to have a magnitude of at least 1.1g to be recognized.
    ///
    /// ```rust,ignore
    /// // let mut lis3dh = ...
    /// let threshold = Threshold::g(Range::G2, 1.1);
    /// lis3dh.configure_irq_threshold(threshold);
    /// ```
    #[doc(alias = "INT1_THS")]
    #[doc(alias = "INT2_THS")]
    pub async fn configure_irq_threshold<I: Interrupt>(
        &mut self,
        _int: I,
        threshold: Threshold,
    ) -> Result<(), Error<CORE::BusError>> {
        self.write_register(I::ths_reg(), threshold.0).await
    }

    /// Get interrupt source. The `interrupt_active` field is true when an interrupt is active.
    /// The other fields specify what measurement caused the interrupt.
    pub async fn get_irq_src<I: Interrupt>(
        &mut self,
        _int: I,
    ) -> Result<InterruptSource, Error<CORE::BusError>> {
        let irq_src = self.read_register(I::src_reg()).await?;
        Ok(InterruptSource::from_bits(irq_src))
    }

    /// Configure 'Sleep to wake' and 'Return to sleep' threshold and duration.
    ///
    /// The LIS3DH can be programmed to automatically switch to low-power mode upon recognition of a determined event.  
    /// Once the event condition is over, the device returns back to the preset normal or highresolution mode.
    ///
    /// Example: enter low-power mode. When a measurement above 1.1g is registered, then wake up
    /// for 25ms to send the data.
    ///
    /// ```rust,ignore
    /// // let mut lis3dh = ...
    ///
    /// let range = Range::default();
    /// let data_rate = DataRate::Hz_400;
    ///
    /// let threshold = Threshold::g(range, 1.1);
    /// let duration = Duration::miliseconds(data_rate, 25.0);
    ///
    /// lis3dh.configure_switch_to_low_power(threshold, duration)?;
    ///
    /// lis3dh.set_datarate(data_rate)?;
    /// ```
    #[doc(alias = "ACT_THS")]
    #[doc(alias = "ACT_DUR")]
    #[doc(alias = "act")]
    pub async fn configure_switch_to_low_power(
        &mut self,
        threshold: Threshold,
        duration: Duration,
    ) -> Result<(), Error<CORE::BusError>> {
        self.write_register(Register::ACT_THS, threshold.0 & 0b0111_1111)
            .await?;
        self.write_register(Register::ACT_DUR, duration.0).await
    }

    /// Reboot memory content
    pub async fn reboot_memory_content(&mut self) -> Result<(), Error<CORE::BusError>> {
        self.register_set_bits(Register::CTRL5, 0b1000_0000).await
    }

    const FIFO_ENABLE_BIT: u8 = 0b0100_0000;

    /// Configures FIFO and then enables it
    pub async fn enable_fifo(
        &mut self,
        mode: FifoMode,
        threshold: u8,
    ) -> Result<(), Error<CORE::BusError>> {
        debug_assert!(threshold <= 0b0001_1111);

        let bits = (threshold & 0b0001_1111) | mode.to_bits();
        self.write_register(Register::FIFO_CTRL, bits).await?;
        self.register_set_bits(Register::CTRL5, Self::FIFO_ENABLE_BIT)
            .await
    }

    /// Disable FIFO. This resets the FIFO state
    pub async fn disable_fifo(&mut self) -> Result<(), Error<CORE::BusError>> {
        self.write_register(Register::FIFO_CTRL, 0x00).await?;
        self.register_clear_bits(Register::CTRL5, Self::FIFO_ENABLE_BIT)
            .await
    }

    /// Get the status of the FIFO
    pub async fn get_fifo_status(&mut self) -> Result<FifoStatus, Error<CORE::BusError>> {
        let status = self.read_register(Register::FIFO_SRC).await?;

        Ok(FifoStatus::from_bits(status))
    }

    /// Get normalized ±g reading from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    pub async fn accel_norm(&mut self) -> Result<F32x3, Error<CORE::BusError>> {
        // The official driver from ST was used as a reference.
        // https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3dh_STdC
        let mode = self.get_mode().await?;
        let range = self.get_range().await?;

        // See "2.1 Mechanical characteristics" in the datasheet to find the
        // values below. Scale values have all been divided by 1000 in order
        // to convert the resulting values from mG to G, while avoiding doing
        // any actual division on the hardware.
        let scale = match (mode, range) {
            // High Resolution mode
            (Mode::HighResolution, Range::G2) => 0.001,
            (Mode::HighResolution, Range::G4) => 0.002,
            (Mode::HighResolution, Range::G8) => 0.004,
            (Mode::HighResolution, Range::G16) => 0.012,
            // Normal mode
            (Mode::Normal, Range::G2) => 0.004,
            (Mode::Normal, Range::G4) => 0.008,
            (Mode::Normal, Range::G8) => 0.016,
            (Mode::Normal, Range::G16) => 0.048,
            // Low Power mode
            (Mode::LowPower, Range::G2) => 0.016,
            (Mode::LowPower, Range::G4) => 0.032,
            (Mode::LowPower, Range::G8) => 0.064,
            (Mode::LowPower, Range::G16) => 0.192,
        };

        // Depending on which Mode we are operating in, the data has different
        // resolution. Using this knowledge, we determine how many bits the
        // data needs to be shifted. This is necessary because the raw data
        // is in left-justified two's complement and we would like for it to be
        // right-justified instead.
        let shift: u8 = match mode {
            Mode::HighResolution => 4, // High Resolution:  12-bit
            Mode::Normal => 6,         // Normal:           10-bit
            Mode::LowPower => 8,       // Low Power:         8-bit
        };

        let acc_raw = self.accel_raw().await?;
        let x = (acc_raw.x >> shift) as f32 * scale;
        let y = (acc_raw.y >> shift) as f32 * scale;
        let z = (acc_raw.z >> shift) as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    pub async fn sample_rate(&mut self) -> Result<f32, Error<CORE::BusError>> {
        Ok(self.get_datarate().await?.sample_rate())
    }

    /// Get raw acceleration data from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    pub async fn accel_raw(&mut self) -> Result<I16x3, Error<CORE::BusError>> {
        let accel_bytes = self.read_accel_bytes().await?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}

pub trait Lis3dhCore {
    type BusError;

    type WriteRegisterFuture<'a>: Future<Output = Result<(), Error<Self::BusError>>> + 'a
    where
        Self: 'a;
    type ReadRegisterFuture<'a>: Future<Output = Result<u8, Error<Self::BusError>>> + 'a
    where
        Self: 'a;
    type ReadAccelBytesFuture<'a>: Future<Output = Result<[u8; 6], Error<Self::BusError>>> + 'a
    where
        Self: 'a;

    fn write_register(&mut self, register: Register, value: u8) -> Self::WriteRegisterFuture<'_>;

    fn read_register(&mut self, register: Register) -> Self::ReadRegisterFuture<'_>;

    fn read_accel_bytes(&mut self) -> Self::ReadAccelBytesFuture<'_>;
}

impl<CORE> Lis3dhCore for Lis3dh<CORE>
where
    CORE: Lis3dhCore,
{
    type BusError = CORE::BusError;

    type WriteRegisterFuture<'a> =
        impl Future<Output = Result<(), Error<Self::BusError>>>+ 'a
        where
            Self: 'a;
    type ReadRegisterFuture<'a> =
        impl Future<Output = Result<u8, Error<Self::BusError>>>+ 'a
        where
            Self: 'a;
    type ReadAccelBytesFuture<'a> =
        impl Future<Output = Result<[u8; 6], Error<Self::BusError>>>+ 'a
        where
            Self: 'a;

    fn write_register(&mut self, register: Register, value: u8) -> Self::WriteRegisterFuture<'_> {
        async move { self.core.write_register(register, value).await }
    }

    fn read_register(&mut self, register: Register) -> Self::ReadRegisterFuture<'_> {
        async move { self.core.read_register(register).await }
    }

    fn read_accel_bytes(&mut self) -> Self::ReadAccelBytesFuture<'_> {
        async move { self.core.read_accel_bytes().await }
    }
}

/// Marker to indicate I2C is used to communicate with the Lis3dh
pub struct Lis3dhI2C<I2C> {
    /// Underlying I²C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

impl<I2C, E> Lis3dhCore for Lis3dhI2C<I2C>
where
    I2C: I2c<Error = E>,
{
    type BusError = E;

    type WriteRegisterFuture<'a> =
    impl Future<Output = Result<(), Error<Self::BusError>>>+ 'a
    where
        Self: 'a;
    type ReadRegisterFuture<'a> =
    impl Future<Output = Result<u8, Error<Self::BusError>>>+ 'a
    where
        Self: 'a;
    type ReadAccelBytesFuture<'a> =
    impl Future<Output = Result<[u8; 6], Error<Self::BusError>>>+ 'a
    where
        Self: 'a;

    /// Write a byte to the given register.
    fn write_register(&mut self, register: Register, value: u8) -> Self::WriteRegisterFuture<'_> {
        async move {
            if register.read_only() {
                return Err(Error::WriteToReadOnly);
            }

            self.i2c
                .write(self.address, &[register.addr(), value])
                .await
                .map_err(Error::Bus)
        }
    }

    /// Read a byte from the given register.
    fn read_register(&mut self, register: Register) -> Self::ReadRegisterFuture<'_> {
        async move {
            let mut data = [0];

            self.i2c
                .write_read(self.address, &[register.addr()], &mut data)
                .await
                .map_err(Error::Bus)
                .and(Ok(data[0]))
        }
    }

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Self::ReadAccelBytesFuture<'_> {
        async move {
            let mut data = [0u8; 6];

            self.i2c
                .write_read(self.address, &[Register::OUT_X_L.addr() | 0x80], &mut data)
                .await
                .map_err(Error::Bus)
                .and(Ok(data))
        }
    }
}

/// Marker to indicate SPI is used to communicate with the Lis3dh
pub struct Lis3dhSPI<SPI> {
    /// Underlying SPI device
    spi: SPI,
}

impl<SPI, ESPI> Lis3dhSPI<SPI>
where
    SPI: SpiDevice<Error = ESPI>,
{
    /// Writes to many registers. Does not check whether all registers
    /// can be written to
    async unsafe fn write_multiple_regs(
        &mut self,
        start_register: Register,
        data: &[u8],
    ) -> Result<(), Error<ESPI>> {
        use embedded_hal::spi::Operation::*;
        self.spi
            .transaction(&mut [Write(&[start_register.addr() | 0x40]), Write(data)])
            .await
            .map_err(Error::Bus)?;
        Ok(())
    }

    /// Read from the registers for each of the 3 axes.
    async fn read_multiple_regs(
        &mut self,
        start_register: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<ESPI>> {
        use embedded_hal::spi::Operation::*;
        self.spi
            .transaction(&mut [Write(&[start_register.addr() | 0xC0]), TransferInPlace(buf)])
            .await
            .map_err(Error::Bus)?;
        Ok(())
    }
}

impl<SPI, ESPI> Lis3dhCore for Lis3dhSPI<SPI>
where
    SPI: SpiDevice<Error = ESPI>,
{
    type BusError = ESPI;

    type WriteRegisterFuture<'a> =
    impl Future<Output = Result<(), Error<Self::BusError>>>+ 'a
    where
        Self: 'a;
    type ReadRegisterFuture<'a> =
    impl Future<Output = Result<u8, Error<Self::BusError>>>+ 'a
    where
        Self: 'a;
    type ReadAccelBytesFuture<'a> =
    impl Future<Output = Result<[u8; 6], Error<Self::BusError>>>+ 'a
    where
        Self: 'a;

    /// Write a byte to the given register.
    fn write_register(&mut self, register: Register, value: u8) -> Self::WriteRegisterFuture<'_> {
        async move {
            if register.read_only() {
                return Err(Error::WriteToReadOnly);
            }
            unsafe { self.write_multiple_regs(register, &[value]).await }
        }
    }

    /// Read a byte from the given register.
    fn read_register(&mut self, register: Register) -> Self::ReadRegisterFuture<'_> {
        use embedded_hal::spi::Operation::*;
        async move {
            let mut data = [0];
            self.spi
                .transaction(&mut [Write(&[register.addr() | 0x80]), TransferInPlace(&mut data)])
                .await
                .map_err(Error::Bus)?;
            Ok(data[0])
        }
    }

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Self::ReadAccelBytesFuture<'_> {
        async move {
            let mut data = [0u8; 6];
            self.read_multiple_regs(Register::OUT_X_L, &mut data)
                .await?;
            Ok(data)
        }
    }
}

/// Sensor configuration options
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Configuration {
    /// The operating mode, default [`Mode::HighResolution`].
    pub mode: Mode,
    /// The output data rate, default [`DataRate::Hz_400`].
    pub datarate: DataRate,
    /// Measure changes in the x axis, default `true`.
    pub enable_x_axis: bool,
    /// Measure changes in the y axis, default `true`.
    pub enable_y_axis: bool,
    /// Measure changes in the z axis, default `true`.
    pub enable_z_axis: bool,
    /// When is data updated
    ///
    /// - when `true`: only after data is read
    /// - when `false`: continually
    ///
    /// default `true`
    pub block_data_update: bool,
    /// Enable temperature measurements. When set, it implies `block_data_update = true`.
    ///
    /// default: `false`
    pub enable_temperature: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            enable_temperature: false,
            block_data_update: true,
            mode: Mode::HighResolution, // Question: should this be normal?
            datarate: DataRate::Hz_400,
            enable_x_axis: true,
            enable_y_axis: true,
            enable_z_axis: true,
        }
    }
}
