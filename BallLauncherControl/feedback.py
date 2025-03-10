import serial
import time

# Configure serial connection (adjust COM port or ttyUSB as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Use '/dev/ttyUSB0' for Linux
time.sleep(2)  # Give time for the Arduino to initialize

def send_command(command):
    """Send a character command to Arduino."""
    ser.write(command.encode())  # Send character
    print(f"Sent: {command}")

def receive_response():
    """Wait for a single-character response from Arduino."""
    while True:
        if ser.in_waiting > 0:
            response = ser.read().decode().strip()
            print(f"Received: {response}")
            return response

# Example usage
send_command('1')  # Start the sequence
response = receive_response()  # Wait for completion signal ('D')

if response == 'D':
    print("Arduino sequence completed!")
    send_command('0')
ser.close()  # Close the connection when done
