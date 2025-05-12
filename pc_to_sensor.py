#!/usr/bin/env python3
"""
A21 Ultrasonic Sensor Interface - USB-RS485
This script communicates with an A21 ultrasonic sensor via USB-RS485 converter
using the Modbus RTU protocol.
"""

import serial
import time
import struct
import sys

# Configuration
PORT = '/dev/ttyUSB0'  # Change this to your USB-RS485 device
BAUDRATE = 115200
SENSOR_ADDRESS = 0x01
READ_INTERVAL = 1.0  # seconds

def calculate_crc(data):
    """Calculate Modbus RTU CRC-16 for the given data"""
    crc = 0xFFFF
    
    for byte in data:
        crc ^= byte
        
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
                
    # Return CRC as bytes (low byte first, then high byte)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def read_distance(ser):
    """Read distance from A21 sensor"""
    # Prepare Modbus RTU command to read register 0x0101 (1 register)
    # [Address][Function][RegisterHigh][RegisterLow][CountHigh][CountLow][CRCLow][CRCHigh]
    command = bytes([SENSOR_ADDRESS, 0x03, 0x01, 0x01, 0x00, 0x01])
    
    # Calculate and append CRC
    crc = calculate_crc(command)
    command += crc
    
    # Clear input buffer before sending command
    ser.reset_input_buffer()
    
    # Send command
    ser.write(command)
    
    # Debug: Print sent command
    cmd_hex = ' '.join([f"{b:02X}" for b in command])
    print(f"Command sent: {cmd_hex}")
    
    # Wait for response
    time.sleep(0.1)
    
    # Read response (expected 7 bytes minimum)
    response = ser.read(20)  # Read up to 20 bytes (more than enough)
    
    if len(response) >= 7:
        # Debug: Print received response
        resp_hex = ' '.join([f"{b:02X}" for b in response])
        print(f"Response received: {resp_hex}")
        
        # Verify response header
        if response[0] == SENSOR_ADDRESS and response[1] == 0x03:
            # Extract data bytes (response[3] is high byte, response[4] is low byte)
            distance = (response[3] << 8) | response[4]
            
            # Verify CRC
            received_data = response[:-2]  # Data without CRC
            received_crc = response[-2:]   # Last two bytes are CRC
            calculated_crc = calculate_crc(received_data)
            
            if received_crc == calculated_crc:
                return distance
            else:
                print("CRC verification failed")
                return None
        else:
            print("Invalid response header")
            return None
    else:
        print("No valid response received")
        return None

def main():
    """Main function"""
    print("A21 Ultrasonic Sensor - USB-RS485 Interface")
    print(f"Connecting to {PORT} at {BAUDRATE} baud...")
    
    try:
        # Open serial port
        with serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        ) as ser:
            print("Connection established.")
            print("Press Ctrl+C to exit.")
            
            last_reading_time = time.time()
            
            while True:
                current_time = time.time()
                
                # Read at specified interval
                if current_time - last_reading_time >= READ_INTERVAL:
                    distance = read_distance(ser)
                    
                    if distance is not None:
                        print(f"Distance: {distance} mm")
                    
                    last_reading_time = current_time
                
                # Small delay to reduce CPU usage
                time.sleep(0.01)
                
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()