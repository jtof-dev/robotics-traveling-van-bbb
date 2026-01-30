#!/bin/bash

# move out of scripts/
cd ..

sudo rm -rf build/

mkdir build

cd build

cmake ..

make

cd ../scripts

