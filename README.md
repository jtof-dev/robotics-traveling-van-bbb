# ball and beam robot

```ascii flowchart
                              ┌─────────┐
                              │         │
                              │ initial │
                              │  boot   │
                              │         │
                              └────┬────┘
                                   │
            ┌───────────────┐      │      ┌─────────────┐
            │               │      │      │             │
            │ reset beam to │◄─────┼─────►│ set up pins │
            │    center     │      │      │ and sensor  │
            │               │      │      │             │
            └───────────────┘      │      └─────────────┘
                                   ▼
                              ┌────────┐
                              │        │
                              │ main() │
                              │        │
                              └────┬───┘
                                   │
                                   │
                                   │
                                   ▼
  ┌────────────────┐      ┌──────────────────┐      ┌───────────────┐
  │                │      │                  │      │               │
  │  send desired  ├─────►│ write a frame to ├─────►│ read time of  │
  │ angle to motor │      │ the touchscreen  │      │ flight sensor │
  │                │      │                  │      │               │
  └────────────────┘      └──────────────────┘      └────────────┬──┘
     ▲                                                           │
     │                                                           │
     │                                                           │
     │                                                           │
     │                                                           │
     │    ┌─────────────────────┐      ┌─────────────────────┐   │
     │    │                     │      │                     │   │
     └────┤ adjust output value │◄─────┤ run PID calculation │◄──┘
          │                     │      │                     │
          └─────────────────────┘      └─────────────────────┘
```

# software

## building

- first, fetch all submodules with `git submodule update --init --recursive` or delete and re-clone all submodules with `scripts/submoduleSetup.sh`

then build with CMAKE (or with `scripts/buildFresh.sh`):

```
    mdkir build
    cd build
    cmake ..
    make
```

(note: updating configurations in `src/configuration.hpp` does not trigger a proper re-build, so only running `make` will often not be enough)

# pins

## VL53l0X time of flight sensor

| **pin**   | **pico pin / rail** | **function** | **notes**                       |
| --------- | ------------------- | ------------ | ------------------------------- |
| **VCC**   | 3.3V rail           | power        |                                 |
| **GND**   | GND rail            | ground       |                                 |
| **SDA**   | GP6 (pin 9)         | I2C1 SDA     | primary data                    |
| **SCL**   | GP7 (pin 10)        | I2C1 SCL     | primary clock                   |
| **XSHUT** | -                   | shutdown     | leave disconnected or pull high |
| **GP101** | -                   | interrupt    | not required for basic ranging  |

## NEMA 17 stepper motor with magnetic encoder

| **pin**  | **wire color** | **function**                               |
| :------- | :------------- | :----------------------------------------- |
| **VCC**  | red            | 3.3V power                                 |
| **EGND** | black          | GND                                        |
| **EA+**  | brown          | channel A pulse (distance/speed)           |
| **EB+**  | blue           | channel B pulse (direction)                |
| **EA-**  | orange         | differential phase A- (noise cancellation) |
| **EB-**  | green          | differential phase B- (noise cancellation) |
| **EZ+**  | yellow         | index pulse (once per revolution)          |
| **EZ-**  | white          | index pulse- (noise cancellation)          |

## TMC2209 stepper motor driver

| **pin**  | **connection source** | **function**  | **notes**                          |
| -------- | --------------------- | ------------- | ---------------------------------- |
| **VM**   | 12V power             | motor voltage | —                                  |
| **GND**  | 12V GND               | power ground  | —                                  |
| **VIO**  | 3.3V (pico)           | logic voltage | —                                  |
| **GND**  | GND (pico)            | logic ground  | —                                  |
| **STEP** | pin 16 (pico)         | step signal   | —                                  |
| **DIR**  | pin 17 (pico)         | direction     | —                                  |
| **EN**   | GND (pico)            | enable        | active low (always ON)             |
| **MS1**  | -                     | microstep 1   | active low (for 1/8 microstepping) |
| **MS2**  | -                     | microstep 2   | active low (for 1/8 microstepping) |
| **A1**   | **A+**                | phase A       | **red**                            |
| **A2**   | **A-**                | phase A       | **black**                          |
| **B1**   | **B+**                | phase B       | **yellow**                         |
| **B2**   | **B-**                | phase B       | **blue**                           |

## pi pico

| **pin**    | **connection**         |
| :--------- | :--------------------- |
| **VIN**    | schottky diode cathode |
| **GND**    | buck converter GND     |
| **3V3**    | to rail on protoboard  |
| **pin 6**  | I2C1 SDA on VL53l0X    |
| **pin 7**  | I2C1 SCL on VL53l0X    |
| **pin 16** | STEP on TMC2209        |
| **pin 17** | DIR on TMC2209         |

- beam is 303mm long
- distance from sensor to end of beam is 13mm

## submodules

- [raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [jtof-dev/pico-pid-library](https://github.com/jtof-dev/pico-pid-library)
- [yspreen/VL53L0X-driver-pico-sdk-cpp](https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp)

# to-do

- [ ] double-check the math used around the PID
