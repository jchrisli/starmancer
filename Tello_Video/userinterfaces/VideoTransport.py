'''
    Send decoded video packets through websockets to a webpage
'''

from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory

from twisted.internet import reactor
import threading
import json

from utils.colorPrint import color_print

class VideoTransportProtocol(WebSocketServerProtocol):
    ## Note this is a class variable
    connection = None

    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):
        VideoTransportProtocol.connection = self
        initMessage = json.dumps({ \
                    'action' : 'init', \
                    'width'  : 960, \
                    'height' : 720, \
        })
        self.sendMessage(initMessage, False)
        print("WebSocket connection open.")

    def onClose(self, wasClean, code, reason):
        VideoTransportProtocol.connection = None
        print("WebSocket connection closed: {0}".format(reason))

    @classmethod
    def broadcast_message(cls, data):
        ## Add the NAL bytes
        #nal = b'\0x00\0x00\0x00\0x01'
        #payload = nal + data
        payload = data
        # color_print('length of data to be sent {0}'.format(len(payload)))
        if VideoTransportProtocol.connection is not None:
            ## Always send binary messages
            #color_print('length of data sent {0}'.format(len(payload)))
            reactor.callFromThread(VideoTransportProtocol.sendMessage, VideoTransportProtocol.connection, payload, True)
'''
    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))

        # echo back message verbatim
        self.sendMessage(payload, isBinary)
'''

class VideoTransport(threading.Thread):
    def __init__(self, port):
        super(VideoTransport, self).__init__()
        self._factory = None
        self._wsPort = port

    def run(self):
        self._factory = WebSocketServerFactory(u"ws://0.0.0.0:" + str(self._wsPort))
        self._factory.protocol = VideoTransportProtocol
        reactor.listenTCP(self._wsPort, self._factory)
        reactor.run(installSignalHandlers=0)
