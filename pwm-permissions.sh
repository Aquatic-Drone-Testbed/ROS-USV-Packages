#!/bin/bash
echo "Initialising Hardware PWM..."
sudo python3 pwm-init.py
sudo chown -R root:gpio /sys/class/pwm
sudo chmod -R 770 /sys/class/pwm
sudo chown -R root:gpio /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip*
sudo chmod -R 770 /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip*
echo "Hardware PWM Ready."