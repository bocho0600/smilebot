import time
from .RP2040 import I2C

I2C.init(bus_number=1, addr=0x08)
class ItemCollectionModule:
    is_initialized = False

    lifter_servo = 3
    lifter_positions = [20, 97, 180]

    gripper_servo = 4
    gripper_speeds = [72, 88, 96]
    # maximum open for 1.5 seconds

    @classmethod
    def init(cls):
        cls.gripper_close(2.5)
        cls.gripper_open(1.7)
        cls.stop_all()

    @classmethod
    def stop_all(cls):
        cls.gripper_stop()
        cls.lifter_set(2)

    @classmethod
    def gripper_close(cls,seconds=1):
        I2C.ServoWrite(cls.gripper_servo, cls.gripper_speeds[0])
        time.sleep(seconds)
        cls.gripper_stop()
        pass

    @classmethod
    def gripper_open(cls,seconds=1):
        I2C.ServoWrite(cls.gripper_servo, cls.gripper_speeds[2])
        time.sleep(seconds)
        cls.gripper_stop()
        pass

    @classmethod
    def gripper_hold (cls,seconds=1):
        I2C.ServoWrite(cls.gripper_servo, cls.gripper_speeds[1])
        time.sleep(seconds)
        cls.gripper_stop()
        pass

    @classmethod
    def gripper_stop(cls):#greater num = open less num = close
        I2C.ServoWrite(cls.gripper_servo, cls.gripper_speeds[1])
        pass

    @classmethod
    def lifter_set(cls,level, seconds=1):
        I2C.ServoWrite(cls.lifter_servo, cls.lifter_positions[level])
        time.sleep(seconds)





