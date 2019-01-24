from websocket_server import WebsocketServer
import json
from utils.colorPrint import color_print 
import threading

class UiWsHandler2():
    def __init__(self, port):
        #self.__clients = []
        self.__port = port
        self.__message_handler = None
        self.__server = None
        self.started = False
        self.__server_thread = None

    def setMessageHandler(self, hand):
        self.__message_handler = hand

    def __handleMessage(self, client, server, message):
        msg = message
        #msg = str(msg)
        #print(msg)
        msgObj = json.loads(msg)
        msgType = msgObj['type']
        msgPayload = msgObj['payload']
        # Viewpoint configuration (position and orientation)
        if msgType == 'set_vp':
            vp_x = float(msgPayload['x'])
            vp_y = float(msgPayload['y'])
            vp_z = float(msgPayload['z'])
            vp_yaw_x = float(msgPayload['yaw_x'])
            vp_yaw_y = float(msgPayload['yaw_y'])
            vp_yaw_z = float(msgPayload['yaw_z'])
            if self.__message_handler is not None:
                self.__message_handler(msgType, {'x': vp_x, 'y': vp_y, 'z': vp_z, 'yaw_x': vp_yaw_x,
                    'yaw_y': vp_yaw_y, 'yaw_z': vp_yaw_z})
        # For local control: send command for the flying camera to take the local person's viewpoint
        elif msgType == 'local_vp':
            if self.__message_handler is not None:
                self.__message_handler(msgType, None)
        elif msgType == 'set_vp_height':
            if self.__message_handler is not None:
                h = float(msgPayload['height'])
                self.__message_handler(msgType, {'height': h })
        elif msgType == 'vp_land':
            # TODO: get the controller send land command to drone
            pass

    # Put the message in the sync queue
    def sendMessage(self, message):
        self.__server.send_message_to_all(message)

    # Start the Websocket server
    def __startServer(self):
        self.__server = WebsocketServer(self.__port, host='0.0.0.0')
        #self.__server.set_fn_new_client(self.__handleConnect)
        #self.__server.set_fn_client_left(self.__handleDisconnect)
        self.__server.set_fn_message_received(self.__handleMessage)
        self.started = True
        self.__server.run_forever()

    def startServer(self):
        self.__server_thread = threading.Thread(target=self.__startServer)
        self.__server_thread.start() 
'''
    def __handleConnect(self, client, server):
        self.__clients.append(client)

    def __handleDisconnect(self, client, server):
        self.__clients.remove(client)
'''