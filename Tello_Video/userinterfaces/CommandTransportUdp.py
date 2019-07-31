import socket
import threading
import json
import traceback

#class CommandTransportUdp(threading.Thread):
class CommandTransportUdp(object):
    def __init__(self, dst_port, src_port, handler):
        super(CommandTransportUdp, self).__init__()
        
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self._sock.settimeout(5.0)
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

    def start(self):
        self._isRunning = True
        self._sock.bind(('0.0.0.0', self._recvPort))
        self._listener_thread = threading.Thread(target=self.listen)
        self._listener_thread.start()

    def listen(self):
        ## Receive data from 
        #self._sock.bind(('0.0.0.0', self._recvPort))
        while self._isRunning:
            try:
                data, addr = self._sock.recvfrom(1024) # buffer size is 1024 bytes
                ## Data will be utf-8 encoded
                json_str = data.decode('utf-8')
                command = json.loads(json_str)
                self._udpHandler(command)
            #except socket.timeout:
            #    print("Command connection timeout - trying again")
            except Exception as e:
                print(traceback.format_exc())
                #print(e)
        print("Vicon connection disconnecting")
        self.close()