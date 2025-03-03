import socket

# Define the UDP IP and Port
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 10111

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        print(f"Received message from {addr}: {data.decode('utf-8')}")
except KeyboardInterrupt:
    print("\nServer shutting down...")
    sock.close()
