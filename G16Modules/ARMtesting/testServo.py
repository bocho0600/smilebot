import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

p = GPIO.PWM(18, 50)
p.start(6.65)

x = 6.65

while x<=7.65:
    print(f"Testing {x}")
    p.ChangeDutyCycle(x)
    time.sleep(0.1)
    x += 0.05

print("stopping")
p.ChangeDutyCycle(7.15)
time.sleep(5)

#print("Rotating backward")
#p.ChangeDutyCycle(2)
#time.sleep(5)

#print("Stopping servo")
#p.ChangeDutyCycle(3)
#time.sleep(5)
print("STOP")
p.stop()
GPIO.cleanup()
