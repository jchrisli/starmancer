import SimpleHTTPServer
import threading
import SocketServer

class UiWebServer():
    def __init__(self, port):
        self.__PORT = port
        self.__handler = SimpleHTTPServer.SimpleHTTPRequestHandler
        self.__server = None
        self.__server_thread = None

    def __start(self):
        self.__server = SocketServer.TCPServer(("", self.__PORT), self.__handler)
        self.__server.serve_forever()

    def startServer(self):
        self.__server_thread = threading.Thread(target=self.__start)
        self.__server_thread.start()
