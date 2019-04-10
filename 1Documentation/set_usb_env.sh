#!/bin/bash

sudo cp ./script/handsfree-serial.rules /etc/udev/rules.d/
sudo service udev reload
sleep 2
sudo service udev restart

sudo usermod -a -G dialout $USER

