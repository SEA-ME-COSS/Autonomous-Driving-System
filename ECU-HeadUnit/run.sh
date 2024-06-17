#!/bin/bash

echo -e '\033[?17;0;0c' > /dev/tty1
./build/HeadUnit -platform linuxfb
