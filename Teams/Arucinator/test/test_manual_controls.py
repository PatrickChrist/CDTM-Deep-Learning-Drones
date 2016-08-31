import cv2 as cv
import libardrone.libardrone as lib_drone
import numpy as np
import asci_keys as keys
import time
import os

drone = lib_drone.ARDrone(True)
delay_time = int(40)  # [ms]
img_manuals = cv.imread(os.path.join("..", "media", "commands.png"))
running = True
cv.imshow("Control window", img_manuals)
flying = False

while running:
    key = cv.waitKey(delay_time)

    if key in [keys.a, keys.A]:
        drone.move_left()
    elif key in [keys.d, keys.D]:
        drone.move_right()
    elif key in [keys.w, keys.W]:
        drone.move_forward()
    elif key in [keys.s, keys.S]:
        drone.move_backward()
    elif key in [keys.i, keys.I]:
        drone.move_up()
    elif key in [keys.m, keys.M]:
        drone.move_down()
    elif key in [keys.j, keys.J]:
        drone.turn_left()
    elif key in [keys.k, keys.K]:
        drone.turn_right()
    elif key in [keys.space]:
        if not flying:
            drone.takeoff()
            flying = True
        else:
            drone.land()
            flying = False
    elif key in [keys.backspace]:
        if not flying:
            drone.halt()
        else:
            drone.land()
            flying = False
            time.sleep(8)
            drone.halt()
        break
    elif key is keys.esc:
        break

drone.halt()