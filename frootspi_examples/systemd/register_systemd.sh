#!/bin/sh

SCRIPT_DIR=$(cd $(dirname $0); pwd)

sudo ln -s ${SCRIPT_DIR}/pigpiod.service /etc/systemd/system
sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service

sudo ln -s ${SCRIPT_DIR}/robot_launch.service /etc/systemd/system
sudo systemctl enable robot_launch.service
sudo systemctl start robot_launch.service

sudo ln -s ${SCRIPT_DIR}/frootspi_netwatcher.service /etc/systemd/system
sudo systemctl enable frootspi_netwatcher.service
sudo systemctl start frootspi_netwatcher.service