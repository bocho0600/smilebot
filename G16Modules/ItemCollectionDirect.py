import RPi.GPIO as GPIO
import time

class ItemCollectionModule:

    is_initialized = False

    @classmethod
    def init(cls):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(12, GPIO.OUT)
        cls.gripper_pin = GPIO.PWM(12,50)


        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        cls.lifter_pin = GPIO.PWM(18,50)

        print("starting")
        cls.gripper_pin.start(6.85)
        cls.lifter_pin.start(7)

        cls.gripper_until_stop = 0
        cls.lifter_until_stop = 0

        cls.t1 = time.time()
        cls.is_initialized = True

    @classmethod
    def gripper_close (cls,seconds):
        cls.gripper_pin.ChangeDutyCycle(7)
        cls.gripper_until_stop = seconds

    @classmethod
    def gripper_open (cls,seconds):
        cls.gripper_pin.ChangeDutyCycle(6.7)
        cls.gripper_until_stop = seconds

    @classmethod
    def gripper_hold (cls,seconds):
        cls.gripper_pin.ChangeDutyCycle(7.4)
        cls.gripper_until_stop = seconds

    @classmethod
    def gripper_stop(cls):#greater num = close less num = open
        cls.gripper_pin.ChangeDutyCycle(6.95)

    @classmethod
    def lifter_up(cls,seconds):
        cls.lifter_pin.ChangeDutyCycle(8)
        cls.lifter_until_stop = seconds

    @classmethod
    def lifter_down(cls,seconds):
        cls.lifter_pin.ChangeDutyCycle(6)
        cls.lifter_until_stop = seconds

    @classmethod
    def lifter_stop(cls):
        cls.lifter_pin.ChangeDutyCycle(7)

    @classmethod
    def update(cls):
        t2 = time.time()
        delta = t2-cls.t1
        cls.t1 = t2


        if cls.gripper_until_stop > 0:
            gripper_until_stop -= delta
            if cls.gripper_until_stop <= 0:
                cls.gripper_stop()

        if cls.lifter_until_stop > 0:
            lifter_until_stop -= delta
            if cls.lifter_until_stop <= 0:
                cls.lifter_stop()

    @classmethod
    def end(cls):
        GPIO.cleanup()
        cls.is_initialized = False
