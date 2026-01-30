# ball and beam robot

## buliding

the system builds with CMAKE, so to build:

```
    mdkir build
    cd build
    cmake ..
    make
```

- then upload the `build/bbb.uf2` file, upload to the board using `upload.sh`

## credits

- [raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [samyarsadat/Pico-PID-Library](https://github.com/samyarsadat/Pico-PID-Library)
- [yspreen/VL53L0X-driver-pico-sdk-cpp](https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp)
  - for the moment, I just cloned the entire repository and placed it inside `pico-sdk`. I'm sure that there is a better way though
