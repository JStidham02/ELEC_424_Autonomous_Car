# This file is used to initialize the Beaglebone PWM

# P9_14 - Speed/ESC - this initializes the main motor ESC to 7.75% duty cycle
with open('/dev/bone/pwm/1/a/period', 'w') as filetowrite:
    filetowrite.write('20000000')
with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
    filetowrite.write('1550000')
with open('/dev/bone/pwm/1/a/enable', 'w') as filetowrite:
    filetowrite.write('1')

# P9_16 - Steering - the initializes the Steering servo to the center (7.5% duty cycle)
with open('/dev/bone/pwm/1/b/period', 'w') as filetowrite:
    filetowrite.write('20000000')
with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
    filetowrite.write('1500000')
with open('/dev/bone/pwm/1/b/enable', 'w') as filetowrite:
    filetowrite.write('1')
