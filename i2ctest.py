from smbus import SMBus
import time
addr = 0x08 # I2C address of the Arduino slave
bus = SMBus(1) # I2C bus (usually 1 for Raspberry Pi)

print("Enter 1 for ON or 0 for OFF")

def string_to_ascii_array(input_string):
    # Create an empty list to hold the ASCII values
    ascii_values = []
    
    # Loop through each character in the input string
    for char in input_string:
        # Append the ASCII value of the character to the list
        ascii_values.append(ord(char))
    
    return ascii_values

while True:
      cmd = input(">>>>   ").strip()  # Get user input and remove any extra spaces
      command = string_to_ascii_array(cmd)
      try:
            print("Sending command: ", command)
            bus.write_i2c_block_data(addr, 0, command)
      except IOError as e:
            print(f"Error communicating with I2C device: {e}")

    # Optionally add a short delay
    #time.sleep(1)