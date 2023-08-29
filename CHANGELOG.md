# 0.8.0
* BREAKING - Now using `embedded-hal-async` version `1.0.0-rc.1`

# 0.7.0
* BREAKING - Now using `embedded-hal-async` version `0.2.0-alpha.1`

# 0.6.1
* Added init functions for creating a driver instance without talking to the device

# 0.6.0
* BREAKING - Now using `embedded-hal-async` version `0.2.0-alpha.0`

# 0.5.1
* Relaxed the bounds requirement for the old accelerometer trait functions
* Added defmt feature

# 0.5.0
* BREAKING - Everything has been made async

# 0.4.2
* Interrupt support, adding methods
    
    - `configure_interrupt_pin`
    - `configure_irq_src_and_control`
    - `configure_irq_src`
    - `configure_irq_duration`
    - `configure_irq_threshold`
    - `get_irq_src`
    - `configure_switch_to_low_power`
    - `configure_irq_src_and_control`

# 0.4.1
* Spi support
* BREAKING - new method has been replaced by new_i2c or new_spi.

# 0.3.0

* BREAKING - `accel_norm` calculations for certain modes were incorrectly applied during 0.2.0 and have been fixed.
* BREAKING - removed Hz_1344_LP5k and LowPower_1K6HZ for safety. If you were affected get in touch in the issues to brainstorm a new api for the next major release that could include these.
* BREAKING - removed `try_into_tracker` fn. See example for how to manually create a tracker.
* provide `get_status` and `is_data_ready` fn.
* enable temperature readings by default and new `get_temp_outf` fn.
* reexport `accelerometer` crate.
