#!/bin/bash

echo "top info"
udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDItop` | grep serial
udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDItop` | grep manufacturer

echo "bottom info"
udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDIbottom` | grep serial
udevadm info -ap `udevadm info -qpath -n /dev/ttyFTDIbottom` | grep manufacturer