from smbus import SMBus
import time
# Made by Kelvin Le, QUT-EGB320-G16
class I2C:

    @classmethod
    def init(cls, bus_number=1, addr=0x08):
        cls.addr = addr  # I2C address of the Arduino slave
        cls.bus = SMBus(bus_number)  # I2C bus (1 for Raspberry Pi, might be different for other devices)
        TRUE_SPEED_MAX = 100
        SET_SPEED_MAX = 255
        cls.prev_left_speed = 0
        cls.prev_right_speed = 0

    # def convert_100_to_255_range(cls,value):
    #     return round((value / TRUE_SPEED_MAX) * SET_SPEED_MAX)


    @classmethod
    def string_to_ascii_array(cls, input_string):  # Function to convert a string to an array of ASCII values to load to I2C bus
        ascii_values = [ord(char) for char in input_string]  # List comprehension for conversion
        return ascii_values

    @classmethod
    def LedWrite(cls, number, state):
        # Validate inputs
        if number not in range(4): # LED numbers start from 0 to 3
            print("Invalid LED number. Please choose between 0 and 3.")
            return

        if state not in ["ON", "OFF"]:
            print("Invalid angle. Please choose between ON and OFF.")
            return

        # Prepare the command string
        command = f"L{number} {state}" #Convert number and state to string

        try:
            # Send the command to the Arduino
            ascii_array = cls.string_to_ascii_array(command)
            cls.bus.write_i2c_block_data(cls.addr, 0, ascii_array)
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")

    @classmethod
    def ServoWrite(cls, number, angle):
        # Validate inputs
        if number not in range(1, 5):  # Servo numbers start from 1 to 4
            print(f"Invalid Servo number: {number}. Please choose between 1 and 4.")
            return

        if angle < 0 or angle > 180:  # Allow any angle within the servo's range
            print(f"Invalid angle: {angle}. Please choose between 0 and 180.")
            return

        # Prepare the command string
        command = f"S{number} {angle}"
        #print(f"Sending command to servo: {command}")

        try:
            # Send the command to the Arduino
            ascii_array = cls.string_to_ascii_array(command)
            cls.bus.write_i2c_block_data(cls.addr, 0, ascii_array)
        except Exception as e:
            print(f"Error sending servo command: {e}")
    
    @classmethod
    def DCWrite(cls, number, direction, speed):
        # Validate inputs
        if number not in range(1, 3): # DC motor numbers start from 1 to 2
            print(f"Invalid DC number: {number}. Please choose between 1 and 2.")
            return
        if speed < 0 or speed > 255:  # Allow any angle within the servo's range
            print(f"Invalid angle: {speed}. Please choose between 0 and 255.")
            return
        if direction not in ["0", "1", "S"]:
            print("Invalid direction. Please choose between 0, 1 or 'S' for STOP.")
            return
        command = f"M{number} {direction} {speed}"
        #print(f"Sending command to motor: {command}")
        try:
            # Send the command to the Arduino
            ascii_array = cls.string_to_ascii_array(command)
            cls.bus.write_i2c_block_data(cls.addr, 0, ascii_array)
        except Exception as e:
            print(f"Error sending servo command: {e}")
    

    @classmethod
    def MoveForward(cls, speed):  # Speed is between 0 and 100
        converted_forward_speed = speed #cls.convert_100_to_255_range(speed)  # Convert to 0-255 for I2C function
        if converted_forward_speed > 0:  # Move forward 
            cls.DCWrite(1, "0", converted_forward_speed)
            cls.DCWrite(2, "0", converted_forward_speed)
        elif converted_forward_speed < 0:
            cls.DCWrite(1, "1", -converted_forward_speed)  # Use the negative value for reverse
            cls.DCWrite(2, "1", -converted_forward_speed)

        elif converted_forward_speed == 0:  # If the speed is 0, stop the motor
            cls.DCWrite(1, "S", 0)
            cls.DCWrite(2, "S", 0)

    @classmethod
    def Rotate(cls, speed):  # Speed is between 0 and 100
        converted_rotational_speed = speed #cls.convert_100_to_255_range(speed)  # Convert to 0-255 for I2C function
        if converted_rotational_speed > 0:
            cls.DCWrite(1, "0", converted_rotational_speed)
            cls.DCWrite(2, "1", converted_rotational_speed)
        elif converted_rotational_speed < 0:
            cls.DCWrite(1, "1", -converted_rotational_speed)  # Use the negative value for reverse
            cls.DCWrite(2, "0", -converted_rotational_speed)

    @classmethod
    def StopAll(cls):
        cls.DCWrite(1, "S", 0)
        cls.DCWrite(2, "S", 0)

    @classmethod
    def SetMotor(cls, motor, speed):
        if speed == 0:
            cls.DCWrite(motor, "S", 0)
        elif speed < 0:
            cls.DCWrite(motor, "1", -speed)
        else:
            cls.DCWrite(motor, "0", speed)

    @classmethod
    def movement(cls, left_speed, right_speed):
        if cls.prev_left_speed != left_speed:  # If the speed is different from the remembered speed
            cls.SetMotor(1, left_speed)
            cls.prev_left_speed = left_speed
        if cls.prev_right_speed != right_speed:  # If the speed is different from the remembered speed 
            cls.SetMotor(2, right_speed)
            cls.prev_right_speed = right_speed

    @classmethod
    def PlaySong(cls, status):
        # Validate inputs

        if status not in ["0", "1"]:
            print("Invalid song status. Please choose between 0, 1.")
            return
        command = f"N{status}"
        print(f"Playing the song...: {command}")
        try:
            # Send the command to the Arduino
            ascii_array = cls.string_to_ascii_array(command)
            cls.bus.write_i2c_block_data(cls.addr, 0, ascii_array)
        except Exception as e:
            print(f"Error sending song command: {e}")
