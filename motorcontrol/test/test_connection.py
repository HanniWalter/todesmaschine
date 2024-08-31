import socket
import struct
import time


status_struct = struct.pack('<H', 42)
movement_struct = struct.pack('<i', 10000)


# Verbindungsdetails
host = '10.0.13.170'  # IP-Adresse des ESP32
#host = '10.0.2.170'  # IP-Adresse des ESP32
port = 80


def send_tcp(type_code, data, listen = False):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(struct.pack('B', type_code)+ data)
        if listen:
            response = s.recv(1024)
            response = struct.unpack('<HBBhh', response[:2])
            return response


#result = send_tcp(1, status_struct, listen=True)
#print(result)

send_tcp(2, movement_struct)

result = send_tcp(1, status_struct, listen=True)
print(result)