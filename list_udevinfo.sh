#!/bin/bash

# echo "top info"
# udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDItop` | grep serial
# udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDItop` | grep manufacturer

# echo "bottom info"
# udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDIbottom` | grep serial
# udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDIbottom` | grep manufacturer


echo "TTYUSB0"
udevadm info -ap `udevadm info -qpath -n /dev/ttyUSB0` | grep serial
udevadm info -ap `udevadm info -qpath -n /dev/ttyUSB0` | grep manufacturer

echo "TTYUSB1"
udevadm info -ap `udevadm info -qpath -n /dev/ttyUSB1` | grep serial
udevadm info -ap `udevadm info -qpath -n /dev/ttyUSB1` | grep manufacturer
