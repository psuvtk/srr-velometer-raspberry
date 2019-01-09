#!/bin/bash

# make sure wiringpi lib has been installed
sudo apt install -y wiringpi

# create deploy folder
if [ ! -d "/home/pi/bin/" ]; then
	mkdir -p /home/pi/bin/
fi

# compile and install 
make
make install
make clean

# make program autostart
sudo -i '#^/home/pi/bin/srr-velometer-raspberry#d' /etc/rc.local
sudo sed -i "#^exit 0#i\/home/pi/bin/srr-velometer-raspberry 1>/home/pi/bin/log.txt 2>&1 &" /etc/rc.local
