import tello
import time

t = tello.Tello('', 8889)
#t.takeoff()  
#time.sleep(8)
#print(t.move_left(1))
#print(t.go(50, 0, 100, 20))
#time.sleep(5)
t.land()