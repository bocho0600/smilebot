from G16Modules.ItemCollection import ItemCollectionModule as ItemCollection
from G16Modules.RP2040 import I2C
import time
if __name__ == "__main__":
    I2C.init()
    ItemCollection.lifter_down(3)
    #time.sleep(2)
    #ItemCollection.lifter_down(10)
    #ItemCollection.gripper_close(1)
    #ItemCollection.gripper_open(5)
    #ItemCollection.collect(17)




