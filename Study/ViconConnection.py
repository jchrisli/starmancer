# Listen to the incoming upd packets from the Vicon server

import socket
import threading
import traceback

class ViconConnection:
    def __init__(self, port, data_handler):
        self._vicon_udp_port = port
        # bind the socket and specify network device
        # not doing anything for now until we figure out the routing
        self._data_handler = data_handler
        self._is_listening = False
        self._receive_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._receive_socket.settimeout(5.0)

    # get the socket ready and start the thread
    def start(self):
        self._is_listening = True
        self._receive_socket.bind(('0.0.0.0', int(self._vicon_udp_port)))
        self._listener_thread = threading.Thread(target=self.listen)
        self._listener_thread.start()
        
    # keep listening to the local socket
    def listen(self):

        while self._is_listening:
            try:
                data = self._receive_socket.recv(66000)
                self._is_listening = self._data_handler(data)
            except socket.timeout:
                print("Vicon connection timeout - trying again")
            except Exception as e:
                print(traceback.format_exc())
                #print(e)
        print("Vicon connection disconnecting")
        self.disconnect()

    def disconnect(self):
        # self._is_listening = False
        try:
            self._receive_socket.close()
        except:
            pass

    def wait(self):
        self._listener_thread.join()