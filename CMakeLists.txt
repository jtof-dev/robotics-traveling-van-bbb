cmake_minimum_required(VERSION 3.20)
project(bbb CXX)

# 1. Find the official Arduino CLI on the host system
find_program(ARDUINO_CLI arduino-cli REQUIRED)

# 2. Setup paths
set(SKETCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/arduino_build)
set(UF2_OUTPUT ${OUTPUT_DIR}/src.ino.uf2)

# 3. Create the configuration string to bypass Bodmer's User_Setup.h
set(TFT_FLAG_LIST
    "-DUSER_SETUP_LOADED=1"
    "-DST7796_DRIVER=1"
    "-DTFT_MISO=16"
    "-DTFT_MOSI=19"
    "-DTFT_SCLK=18"
    "-DTFT_CS=20"
    "-DTFT_DC=22"
    "-DTFT_RST=21"
    "-DLOAD_GLCD=1"
    "-DLOAD_FONT2=1"
    "-DLOAD_FONT4=1"
    "-DLOAD_FONT6=1"
    "-DLOAD_FONT7=1"
    "-DLOAD_FONT8=1"
    "-DLOAD_GFXFF=1"
    "-DSMOOTH_FONT=1"
    "-DTFT_SPI_PORT=0"
    "-DSPI_FREQUENCY=20000000"
    "-DSPI_READ_FREQUENCY=20000000"
    "-DSPI_TOUCH_FREQUENCY=2500000"
)
string(JOIN " " TFT_DEFINES ${TFT_FLAG_LIST})

# 4. Tell CMake to call Arduino CLI to do the heavy lifting
add_custom_command(
    OUTPUT ${UF2_OUTPUT}
    COMMAND ${ARDUINO_CLI} compile 
            --fqbn rp2040:rp2040:rpipico 
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/tft_espi
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/pid        # Automatically pulls in your PID library
            --library ${CMAKE_CURRENT_SOURCE_DIR}/lib/VL53L0X    # Automatically pulls in your TOF library
            --output-dir ${OUTPUT_DIR}
            --build-property "build.extra_flags=${TFT_DEFINES}"
            ${SKETCH_DIR}
    DEPENDS ${SKETCH_DIR}/src.ino
    COMMENT "Delegating bbb firmware compilation to arduino-cli..."
)

# 5. Create the main target that ties it all together
add_custom_target(bbb ALL DEPENDS ${UF2_OUTPUT})
