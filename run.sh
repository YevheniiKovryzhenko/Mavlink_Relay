#!/bin/bash
BUILD_DIR=build
if [ ! -d "$BUILD_DIR" ]; then
    mkdir $BUILD_DIR
else
    rm -rf $BUILD_DIR/*
fi

cmake -S . -B $BUILD_DIR
cd $BUILD_DIR
make

sudo ./Mavlink_Relay -m 192.168.1.1 -mI 14 -mY -td /dev/ttyUSB0 -tb 230400 -ru 192.168.1.2 -pt