I2C application (app/i2c-app.c)
===============================

@tableofcontents

I2C application demonstrates how to access/manipulate Always On (AON) I2C from
the SPE/AON processor. I2C 2, 8 and 10 are in AON domain for both
Jetson TX2 and Jetson AGX.

The `ENABLE_I2C_APP` flag in the `soc/*/target_specific.mk` file
controls compilation of this demo app.

@note It is required to compile device tree and flash board to have them updated
after making kernel device tree changes as described in following sections.

# Jetson TX2 #

The sample app uses I2C bus 2 and BMI160 sensor module for demo purposes. Make
sure I2C bus 2 is disabled in linux kernel as shown below in tegra186-quill-p3310-1000-c03-00-base.dts
device tree file.

```
i2c@c240000 {
        status = "disabled";
};
```

In addition to above device tree file, make sure bus@1 is being removed from the
tegra186-quill-p3310-1000-a00-plugin-manager.dtsi file as well as shown in below
code snippet.

```
bus@1 {
        i2c-bus = <&gen2_i2c>;
        eeprom@0 {
                slave-address = <0x51>;
        };
};
```

BMI160 module is similar to the sample app available at:

https://hackspark.fr/en/electronics/1341-6dof-bosch-6-axis-acceleration-gyro-gravity-sensor-gy-bmi160.html

The I2C app accesses the 40 pin header J21 and expects I2C external module at the pin map
shown below:

- Pin 1 - 3.3V
- Pin 3 - SDA
- Pin 5 - SCL
- Pin 39, 25 - GND

The demo app reads BMI160 sensor ID. If the app successfully retrieves the
correct ID, it prints the following message:

    I2C test successful, sendor ID: 0xd1

@note BMI160 is configured to have the I2C address 0x68 by connecting its SDO pin to
GND.

# Jetson AGX #

The sample app uses I2C bus 8 and on board audio codec chip for demo purposes.
Make sure access to codec chip and I2C bus 8 from other software stacks like
kernel/bootloader etc... is disabled. For kernel, related dts is
tegra194-audio-p2822-0000.dtsi and below is the sample snippet to disable:

```
i2c@c250000 {
        status = "disabled";
        rt5658: rt5659.7-001a@1a {
                compatible = "realtek,rt5658";
                reg = <0x1a>;

                realtek,jd-src = <RT5659_JD_NULL>;
                realtek,dmic1-data-pin = <RT5659_DMIC1_NULL>;
                realtek,dmic2-data-pin = <RT5659_DMIC2_DATA_IN2P>;

                gpios = <&tegra_main_gpio TEGRA194_MAIN_GPIO(S, 5) 0>;

                status = "disabled";
        };
};
```

The demo app reads device ID. Test is successful if app retrieves correct ID and
should print "I2C test successful, sendor ID: 0x1163".
