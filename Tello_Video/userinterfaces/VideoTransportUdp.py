import socket
#import threading

class VideoTransportUdp():
    def __init__(self, port):
        #super(VideoTransportUdp, self).__init__()
        
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._targetPort = port

    def send(self, data):
        self._sock.sendto(data, ('127.0.0.1', self._targetPort))
        #print('Sending {0} bytes to {1}'.format(len(data), self._targetPort))