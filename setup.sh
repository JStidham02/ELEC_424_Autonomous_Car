sudo xauth add $(xauth -f ~debian/.Xauthority list|tail -1)
sudo python3 pwm_init.py
sudo depmod
sudo modprobe encoder_driver
