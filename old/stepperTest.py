import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
control_pins = [17, 27]

for pin in control_pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

seq1 = [
  [1,0],
  [0,0],
]

seq2 = [
  [1,1],
  [0,1],
]

for i in range(5000):
  for halfstep in range(2):
    for pin in range(2):
      GPIO.output(control_pins[pin], seq1[halfstep][pin])
  time.sleep(0.000001)

for i in range(5000):
  for halfstep in range(2):
    for pin in range(2):
      GPIO.output(control_pins[pin], seq2[halfstep][pin])
  time.sleep(0.000001)
    
GPIO.cleanup()
