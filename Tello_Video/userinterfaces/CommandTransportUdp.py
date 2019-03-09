import socket
import threading
import json

class CommandTransportUdp(threading.Thread):
    def __init__(self, dst_port, src_port, handler):
        super(CommandTransportUdp, self).__init__()
        
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._targetPort = dst_port
        self._recvPort = src_port
        self._udpHandler = handler
        self._isRunning = False

    def send(self, data):
        self._sock.sendto(data, ('127.0.0.1', self._targetPort))
        # print('Sending {0} bytes to {1}'.format(len(data), self._targetPort))

    def close(self):
        self._isRunning = False
        self._sock.close()

    def run(self):
        ## Receive data from 
        self._isRunning = True
        self._sock.bind(('127.0.0.1', self._recvPort))
        while self._isRunning:
            data, addr = self._sock.recvfrom(1024) # buffer size is 1024 bytes
            ## Data will be utf-8 encoded
            json_str = data.decode('utf-8')
            command = json.loads(json_str)
            self._udpHandler(command)