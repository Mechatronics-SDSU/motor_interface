import serial
import struct
import numpy
"""
usb_port = "COM6"
baud_rate = 115200

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
"""
def test_tx(num_array): #DELETEME
    print(num_array)