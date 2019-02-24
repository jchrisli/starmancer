import tello
from tello_control_ui import TelloUI
from tello_controller import TelloController


def main():

    drone = tello.Tello('', 8889)  
    ## Maybe we do not need to decode h264 locally, just forward it via gstreamer to the remote comp
    
	# start the Tkinter mainloop
    controller = TelloController(drone)
    controller.start()
    #vplayer = TelloUI(drone,"./img/")
    #vplayer.root.mainloop() 

if __name__ == "__main__":
    main()
