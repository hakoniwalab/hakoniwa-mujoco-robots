#!/bin/bash

if [ $# -eq 1 -a $1 == "clean" ]; then
    rm -rf src/cmake-build/*
    exit 0
fi

cd src/cmake-build
cmake ..
make