import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
control_pins = [3, 4]

for pin in control_pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

seq = [
  [1,0],
  [0,0],
]

for i in range(2000):
  for halfstep in range(2):
    for pin in range(2):
      GPIO.output(control_pins[pin], seq[halfstep][pin])
    time.sleep(0.001)
    
GPIO.cleanup()
