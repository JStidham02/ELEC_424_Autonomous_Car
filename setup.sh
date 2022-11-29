sudo xauth add $(xauth -f ~debian/.Xauthority list|tail -1)
python3 pwm_init.py
