import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
p = GPIO.PWM(18,50)

print("starting")
p.start(7)

def up(seconds):
    p.ChangeDutyCycle(8)
    time.sleep(seconds)
def down(seconds):
    p.ChangeDutyCycle(6)
    time.sleep(seconds)


def stop():
    p.ChangeDutyCycle(7)


if __name__ == "__main__":
    #about 3.5 sec down for every 6 sec up

    down(2)
    #up(7.5)
    stop()
