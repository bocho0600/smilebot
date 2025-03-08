import RPi.GPIO as GPIO# Import the GPIO module
import time # Import the time module
GPIO.setmode(GPIO.BCM)# Set the GPIO pin naming convention to BCM
GPIO.setup(18,GPIO.OUT)# Set up GPIO pin 18 as an output

while True:
      p = GPIO.PWM(18, 50)
      p.start(60) 
      time.sleep(5)
      p.ChangeDutyCycle(50)
      time.sleep(5)
      p.ChangeDutyCycle(40)
      GPIO.cleanup()# Exit the GPIO session cleanly