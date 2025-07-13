# MAX31760 Fan Speed Controller Driver - Ported to Linux v3.18

## Overview

This project provides a port of the MAX31760 fan speed controller driver from the modern Linux kernel (v6.14) to the legacy v3.18 kernel environment. This allows the hardware to be used on older embedded systems or distributions running the 3.18 kernel.

## File Structure

- `v6.14_max31760.c`: Original v6.14 kernel driver
- `max31760.c`: Ported driver for v3.18.24 kernel
- `README.md`: This document

## Major Porting Changes

### 1. Hwmon Registration (Modern vs. Legacy)
The modern `hwmon_chip_info` and generic `read`/`write` operations were replaced with the legacy approach of using individual sysfs `show`/`store` functions for each attribute.

- **v6.14:** Used `devm_hwmon_device_register_with_info`.
- **v3.18:** Switched to `devm_hwmon_device_register_with_groups` and defined all attributes manually using `SENSOR_DEVICE_ATTR`.

### 2. API and Macro Compatibility
Adapted the code to handle APIs and C language features not present in v3.18.

- **C99 to C89:** Modified `for` loops to declare loop variables at the start of the function block.
- **Unavailable Macros:** Replaced macros like `GENMASK`, `FIELD_GET`, and `clamp_val` with direct bitwise operations or `min_t`/`max_t` equivalents.
- **Regmap Helpers:** Replaced newer helper functions (e.g., `regmap_set_bits`) with the core `regmap_update_bits` API.

## Compilation and Installation

### Dependencies
- Linux kernel v3.18.24 source code
- GCC compiler
- Make build system

### Compilation Method
```bash
# Navigate to kernel source directory
cd /path/to/linux-3.18.24

# Copy driver to drivers/hwmon/ directory
cp /path/to/max31760.c drivers/hwmon/

# Add driver to Makefile (drivers/hwmon/Makefile)
# obj-$(CONFIG_SENSORS_MAX31760) += max31760.o

# Enable driver in kernel configuration
make menuconfig
# Device Drivers -> Hardware Monitoring Support -> MAX31760 fan speed controller

# Compile
make modules
```

## Usage

### Device Tree Configuration
```dts
&i2c0 {
    max31760@50 {
        compatible = "adi,max31760";
        reg = <0x50>;
    };
};
```

### sysfs Interface
Once the driver is successfully loaded and bound to the device, it creates a set of files under `/sys/class/hwmon/hwmon*/`. You can use these files to monitor and control the device.

**Temperature Sensors:**
- `temp1_input`: (Read-only) Remote sensor temperature in millidegrees Celsius.
- `temp1_max`: (Read/Write) Maximum temperature threshold.
- `temp1_crit`: (Read/Write) Critical temperature threshold.
- `temp1_max_alarm`: (Read-only) Alarm flag for maximum temperature.
- `temp1_crit_alarm`: (Read-only) Alarm flag for critical temperature.
- `temp1_fault`: (Read-only) Sensor fault flag.
- `temp1_label`: (Read-only) Sensor label ("remote").

*(Files with `temp2_*` correspond to the local (internal) sensor)*

**Fan Control:**
- `fan1_input`: (Read-only) Fan speed in RPM.
- `fan1_fault`: (Read-only) Fan fault alarm flag.
- `fan1_enable`: (Read/Write) Enable/disable the fan monitoring channel (1=on, 0=off).

*(Files with `fan2_*` correspond to the second fan channel)*

**PWM Controller:**
- `pwm1_input`: (Read/Write) Sets the PWM duty cycle (0-255) in manual mode.
- `pwm1_enable`: (Read/Write) Sets the control mode (1: manual, 2: automatic fan speed control).
- `pwm1_freq`: (Read/Write) Sets the PWM output frequency.
- `pwm1_auto_channels_temp`: (Read/Write) Selects the temperature source for automatic mode (1: temp1, 2: temp2, 3: both).
- `pwm1_auto_point_temp_hyst`: (Read/Write) Sets the temperature hysteresis for automatic mode.
- `pwm1_auto_point[1-48]_pwm`: (Read/Write) Defines the 48-point lookup table for temperature-to-PWM mapping in automatic mode.

## Limitations

1. **v3.18.24 Kernel Constraints**: Some features from newer kernels are not supported
2. **HWMON API**: Limited HWMON API in v3.18.24 restricts some functionality
3. **Regmap Features**: Some advanced regmap features are not available

## License

GPL-2.0-only

## Authors

- **Original Driver:** Ibrahim Tilki <Ibrahim.Tilki [at] analog.com>
- **v3.18 Port:** hs.choi <hs.choi [at] piolink.com>

## Change History

- 2025-07-07: v3.18.24 kernel porting completed
- 2023: Original v6.14 driver release 
