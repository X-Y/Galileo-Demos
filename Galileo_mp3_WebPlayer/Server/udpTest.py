import socket

UDP_IP = "192.168.1.162"
UDP_PORT = 5006
MESSAGE = "Hello, World!"


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(bytes(MESSAGE,"utf-8"), (UDP_IP, UDP_PORT))