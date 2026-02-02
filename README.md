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

## submodules

- [raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [jtof-dev/pico-pid-library](https://github.com/jtof-dev/pico-pid-library)
- [yspreen/VL53L0X-driver-pico-sdk-cpp](https://github.com/yspreen/VL53L0X-driver-pico-sdk-cpp)
