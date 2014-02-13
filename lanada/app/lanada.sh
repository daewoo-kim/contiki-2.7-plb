#! /bin/sh

make clean
make $1
msp430-objcopy $1.z1 -O ihex $1.ihex
sudo make $1.upload
