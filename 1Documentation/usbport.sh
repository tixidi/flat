#!/bin/bash

echo 'KERENL=="ttyUSB*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d",ATTRS{idProduct}=="c52f",MODE:="0777",SYMLINK+="ttyros1"' > /etc/udev/rules.d/ttyUSB00001.rules
echo 'KERENL=="ttyUSB*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0bda",ATTRS{idProduct}=="5411",MODE:="0777",SYMLINK+="ttyros2"' > /etc/udev/rules.d/ttyUSB00002.rules
echo 'KERENL=="ttyUSB*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1d6b",ATTRS{idProduct}=="0002",MODE:="0777",SYMLINK+="ttyros3"' > /etc/udev/rules.d/ttyUSB00003.rules


service udev reload
sleep 2
service udev restart
