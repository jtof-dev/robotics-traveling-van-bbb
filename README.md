# ball and beam robot

## building

- first, fetch all submodules with `git submodule update --init --recursive`

then build with CMAKE:

```
    mdkir build
    cd build
    cmake ..
    make
```

- then upload the `build/bbb.uf2` file, upload to the board using `upload.sh`

# notes to self

## pins

| pin | function | description             |
| --- | -------- | ----------------------- |
| 4   | I2C SDA  | ToF sensor              |
| 5   | I2C SCL  | ToF sensor              |
| 14  | GPIO     | stepper motor direction |
| 15  | GPIO     | stepper motor step      |

- for the motor driver, connect MS1 and MS2 HIGH (3.3V)

## submodules

- [raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [jtof-dev/pico-pid-library](https://github.com/jtof-dev/pico-pid-library)
- [yspreen/VL53L0X-driver-pico-sdk-cpp](https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp)

# to-do

- [ ] double-check the math used around the PID
