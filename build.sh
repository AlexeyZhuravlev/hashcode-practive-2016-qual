#!/bin/bash

mkdir -p build
cd build
cmake ..

if [ $# -ne 1 ]; then
  make
else
  make $1
fi