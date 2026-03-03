# ball and beam robot

## building

- first, fetch all submodules with `git submodule update --init --recursive`

then build with CMAKE (or use `scripts/buildFresh.sh`):

```
    mdkir build
    cd build
    cmake ..
    make
```

- then upload the `build/bbb.uf2` file, upload to the board using `upload.sh`

# pins

## VL53l0X time of flight sensor

| **component pin** | **pico pin / rail** | **function** | **notes**                       |
| ----------------- | ------------------- | ------------ | ------------------------------- |
| **VCC**           | 3.3V rail           | power        |                                 |
| **GND**           | GND rail            | ground       |                                 |
| **SDA**           | GP6 (pin 9)         | I2C1 SDA     | primary data                    |
| **SCL**           | GP7 (pin 10)        | I2C1 SCL     | primary clock                   |
| **XSHUT**         | unused              | shutdown     | leave disconnected or pull high |
| **GP101**         | unused              | interrupt    | not required for basic ranging  |

## TMC2208 stepper motor driver

| **driver pin** | **connection source** | **function**  | **wire color (motor)** |
| -------------- | --------------------- | ------------- | ---------------------- |
| **VM**         | 12V power             | motor voltage | —                      |
| **GND**        | PSU GND               | power ground  | —                      |
| **VIO**        | 3.3V (pico)           | logic voltage | —                      |
| **GND**        | GND (pico)            | logic ground  | —                      |
| **STEP**       | GP17 (pin 22)         | step signal   | —                      |
| **DIR**        | GP16 (pin 21)         | direction     | —                      |
| **EN**         | GND (pico)            | enable        | active low (always ON) |
| **MS1**        | 3.3V (pico)           | microstep 1   | active high            |
| **MS2**        | 3.3V (pico)           | microstep 2   | active high            |
| **M1A**        | **A+**                | phase A       | **black**              |
| **M2A**        | **A-**                | phase A       | **green**              |
| **M1B**        | **B+**                | phase B       | **red**                |
| **M2B**        | **B-**                | phase B       | **blue**               |

## pi pico

| **pico pin** | **label** | **connection**           | **purpose**        |
| ------------ | --------- | ------------------------ | ------------------ |
| **pin 40**   | VBUS      | schottky diode (anode)   | USB power output   |
| **pin 39**   | VSYS      | schottky diode (cathode) | main system input  |
| **pin 38**   | GND       | GND rail                 | common ground      |
| **pin 36**   | 3V3       | 3.3V rail                | logic power output |

## submodules

- [raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [jtof-dev/pico-pid-library](https://github.com/jtof-dev/pico-pid-library)
- [yspreen/VL53L0X-driver-pico-sdk-cpp](https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp)

# to-do

- [ ] double-check the math used around the PID
