from rpi_hardware_pwm import HardwarePWM

pi_1 = HardwarePWM(pwm_channel=0, hz=50, chip=0) # connect to pi gpio pin 12
pi_2 = HardwarePWM(pwm_channel=1, hz=50, chip=0) # connect to pi gpio pin 13
