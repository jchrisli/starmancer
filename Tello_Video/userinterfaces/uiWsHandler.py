import asyncio
import websockets
import json
import threading
import janus

# Handle WebSocket messages from the frontend
class UiWsHandler():
    def __init__(self, port):
        self.__port = port
        self.__serve_ws = None
        self.__connected = None
        self.__event_loop_thread = None
        self.__message_handler = None
        self.__queue = None

    def setMessageHandler(self, hand):
        self.__message_handler = hand

    async def __handleMessage(self, websocket, path):
        self.__connected = websocket
        while True:
            msg = await websocket.recv()
            msg = str(msg)
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

    async def __sendMessageInternal(self):
        # Keep sending messages
        while True:
            # TODO: check if the queue is empty?
            message = await self.__queue.async_q.get()
            if self.__connected is not None:
                await self.__connected.send(message)

    # Put the message in the sync queue
    def sendMessage(self, message):
        self.__queue.sync_q.put(message)

    # Start the Websocket server
    def __startServer(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.__serve_ws = websockets.serve(self.__handleMessage, '0.0.0.0', self.__port)
        # Initiate the queue
        self.__queue = janus.Queue(loop=asyncio.get_event_loop())
        tasks = [self.__sendMessageInternal(), self.__serve_ws]
        asyncio.get_event_loop().run_until_complete(asyncio.wait(tasks))
        # TODO: terminate the server upon some signal
        # asyncio.get_event_loop().run_forever()

    # Wrap the server event loop in a thread
    def startServer(self):
        self.__event_loop_thread = threading.Thread(target=self.__startServer)
        self.__event_loop_thread.start()

        
