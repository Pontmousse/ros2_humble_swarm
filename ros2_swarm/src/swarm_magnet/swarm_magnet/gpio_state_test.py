import RPi.GPIO as GPIO

pin = 4  # GPIO pin number

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

GPIO.output(pin, GPIO.LOW)

state = GPIO.input(pin)
print(f"GPIO {pin} state is", "HIGH" if state else "LOW")


# run
# cd ~/ros2_swarm/src/swarm_magnet/swarm_magnet
# python3 gpio_state_test.py
       
