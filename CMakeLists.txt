cmake_minimum_required(VERSION 3.20)
project(bbb CXX)

find_program(ARDUINO_CLI arduino-cli REQUIRED)

set(SKETCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/arduino_build)
set(UF2_OUTPUT ${OUTPUT_DIR}/src.ino.uf2)

set(TFT_FLAG_LIST
    "-DUSER_SETUP_LOADED=1"
    "-DST7796_DRIVER=1"
    "-DTFT_MISO=0"
    "-DTFT_MOSI=19"
    "-DTFT_SCLK=18"
    "-DTFT_CS=21"
    "-DTFT_DC=22"
    "-DTFT_RST=20"
    "-DLOAD_GLCD=1"
    "-DLOAD_FONT2=1"
    "-DLOAD_FONT4=1"
    "-DLOAD_FONT6=1"
    "-DLOAD_FONT7=1"
    "-DLOAD_FONT8=1"
    "-DLOAD_GFXFF=1"
    "-DSMOOTH_FONT=1"
    "-DTFT_SPI_PORT=0"
    "-DSPI_FREQUENCY=10000000"
    "-DSPI_READ_FREQUENCY=20000000"
    "-DSPI_TOUCH_FREQUENCY=2500000"
)
string(JOIN " " TFT_DEFINES ${TFT_FLAG_LIST})

add_custom_command(
    OUTPUT ${UF2_OUTPUT}
    COMMAND ${ARDUINO_CLI} compile
            --fqbn rp2040:rp2040:rpipico
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/TFT_eSPI
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/pid
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/VL53L0X
            --output-dir ${OUTPUT_DIR}
            --build-property "build.extra_flags=${TFT_DEFINES}"
            ${SKETCH_DIR}
    DEPENDS ${SKETCH_DIR}/src.ino
    COMMENT "delegating bbb firmware compilation to arduino-cli..."
)

add_custom_target(bbb ALL DEPENDS ${UF2_OUTPUT})
