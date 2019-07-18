import tello
import time

t = tello.Tello('', 8889)

#t.takeoff()
#time.sleep(3)
#for i in range(0, 200):
#    t.rc(0, 0, 0, 0.3)
#    time.sleep(20 / 1000.0)


t.land()