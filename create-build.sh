#!/bin/bash

set -e

self="$(readlink -f "$0")"
cd "$(dirname "${self}")"

mkdir -p build
cd build
cmake ..
make -j
ctest -V
