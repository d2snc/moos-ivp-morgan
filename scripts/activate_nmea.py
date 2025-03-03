import serial

def calculate_checksum(sentence):
    """Calculate NMEA checksum."""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)  # XOR each character
    return f"{checksum:02X}"  # Return hex checksum

def send_command(port, baudrate, command):
    """Send a command to VN-300 over serial."""
    with serial.Serial(port, baudrate, timeout=1) as ser:
        # Calculate checksum
        checksum = calculate_checksum(command)
        full_command = f"${command}*{checksum}\r\n"
        
        print(f"Sending: {full_command}")
        ser.write(full_command.encode('utf-8'))
        
        # Read response
        response = ser.readline().decode('utf-8').strip()
        print(f"Response: {response}")

if __name__ == "__main__":
    serial_port = "/dev/ttyUSB0"  # Change if necessary (e.g., COM3 for Windows)
    baudrate = 115200  # Adjust according to VN-300 settings
    command = "VNWRG,101,1,5,0,0,203"  # Command without '$' and '*XX'
    
    send_command(serial_port, baudrate, command)