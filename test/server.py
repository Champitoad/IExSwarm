#!/usr/bin/python

from bluetooth import *
import random as rd

server_sock = BluetoothSocket(RFCOMM)
server_sock.bind(("", PORT_ANY))
server_sock.listen(1)

advertise_service(server_sock, "IExSwarm", service_classes = [SERIAL_PORT_CLASS], profiles = [SERIAL_PORT_PROFILE])

client_sock, client_info = server_sock.accept()
print ("Connexion de", client_info)

while True :
    data = client_sock.recv(1024)
    print(data)

client_sock.close()
server_sock.close()
