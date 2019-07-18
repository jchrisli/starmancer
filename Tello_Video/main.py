import tello
from tello_control_ui import TelloUI
from tello_controller import TelloController
import sys
from colorama import init


def main():
    init(strip=False)

    drone = tello.Tello('', 8889)  
    
    debug_ui = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'uionly':
            debug_ui = True
    controller = TelloController(drone, debug_ui)
    controller.start()
    #vplayer = TelloUI(drone,"./img/")
    #vplayer.root.mainloop() 

if __name__ == "__main__":
    main()
