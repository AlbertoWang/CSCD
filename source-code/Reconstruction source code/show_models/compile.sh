#!/bin/sh
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make -j4
