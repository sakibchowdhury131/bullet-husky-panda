import serial
import time

# Configure serial connection (adjust COM port or ttyUSB as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Use '/dev/ttyUSB0' for Linux
time.sleep(1)  # Give time for the Arduino to initialize

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
command = input("enter the command")
send_command(command)  # Start the sequence
while True:
    response = receive_response()  # Wait for completion signal ('D')

    if response == 'D':
        print(f"Ball thrown at: {time.time()}")
        send_command('0')
        break
ser.close()  # Close the connection when done
