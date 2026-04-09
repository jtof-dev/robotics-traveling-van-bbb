#!/bin/bash

pwd=$(pwd)
repoRoot=$(git rev-parse --show-toplevel)

cd "$repoRoot"

sudo rm -rf build/

mkdir build

cd build

cmake ..

make -j12

cd "$pwd"
