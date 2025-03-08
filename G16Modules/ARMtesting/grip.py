import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12,50)

print("starting")
p.start(6.85)

def close (seconds):
    p.ChangeDutyCycle(7)
    time.sleep(seconds)
def open (seconds):
    p.ChangeDutyCycle(6.7)
    time.sleep(seconds)

def hold (seconds):
    p.ChangeDutyCycle(7.4)
    time.sleep(seconds)

def stop():#greater num = close less num = open
    p.ChangeDutyCycle(6.95)

if __name__ == "__main__":

    open(1)
    close(1)
    hold(2)
    open(1)
    stop()
    time.sleep(5)

    open(1)
    close(1)
    hold(2)
    open(1)
    stop()
    time.sleep(5)

    open(1)
    close(1)
    hold(2)
    open(1)
    stop()
    time.sleep(5)