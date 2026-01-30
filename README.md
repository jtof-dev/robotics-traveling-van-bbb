- using brettb's PID library, ported to the Pi Pico by [samyarsadat](https://github.com/samyarsadat/Pico-PID-Library)

the system builds with CMAKE, so to build:

```
    mdkir build
    cd build
    cmake ..
    make
```

- aside from `cmake`, the only other package needed is `pico-sdk` (aur)

after generating the bbb.uf2 file, upload using `upload.sh`
