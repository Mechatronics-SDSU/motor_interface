import serial
import struct
import numpy

baud_rate = 115200
usb_port = None
srl = None

try:
    usb_port = "COM6"
    srl = serial.Serial(usb_port, baud_rate)
except:
    try:
        usb_port = "/dev/tty.usbmodem205939804E301" # STM32 Virtual ComPort
        # Initialize the serial connection
        srl = serial.Serial(usb_port, baud_rate)
    except:
        print("unable to connect")


# Initialize the serial connection
srl = serial.Serial(usb_port, baud_rate)

def usb_transmit(num_array):
    # Create a list to hold the packed data
    packed_data = b''
    
    for num in num_array:
        # Pack the number as a 32-bit signed integer (little-endian)
        packed_data += struct.pack('<i', num)

    # Transmit the packed data over USB
    srl.write(packed_data)
    print(f"Transmitted: {num_array}")

def print_tx(num_array): #DELETEME
    print(num_array)

#usb_transmit([100, 0, 0, 0, 0, 0, 0, 0])