import libardrone.libardrone as lib_drone
import asci_keys as keys
import time
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import os
import math
import pygame
import pygame.surfarray


class DroneController:
    flying = False
    loop_running = True
    drone_feed_window = "Drone feed"
    cycle_time = int(40)  # [ms]
    nr_cycles_no_key_pressed = 0
    view_front_camera = True
    turn = 0
    automatic_mode = False
    center = None
    corners = None
    aruco_found = False
    lag_counter = 0
    p_x = 0.8
    d_x = 0.9
    p_y = 0.8
    d_y = 0.9

    def __init__(self, use_webcam=False):
        self.use_webcam = use_webcam
        if self.use_webcam:
            self.cam = cv.VideoCapture(0)
        self.drone = lib_drone.ARDrone2(hd=True)
        time.sleep(1)
        self.time = time.time()
        self.integral = {"err_x": 0, "err_y": 0, "err_height": 0, "err_distance": 0}
        self.last = {"err_x": 0, "err_y":0, "err_height": 0, "err_distance": 0}
        self.control = {"x": 0, "y":0, "height": 0, "distance": 0}
        self.ref_height = 800  # [mm]
        self.height = 0  # [mm]
        self.drone.set_camera_view(True)
        self.battery_level = self.drone.navdata.get(0, dict()).get('battery', 0)

        # Initialize pygame
        pygame.init()
        self.image_shape = self.drone.image_shape  # (720, 1280, 3) = (height, width, color_depth)
        self.marker_size = 0
        self.ref_marker_size = 150
        self.img = np.array([1], ndmin=3)
        self.screen = pygame.display.set_mode((self.image_shape[1], self.image_shape[0]))  # width, height
        self.img_manuals = pygame.image.load(os.path.join("media", "commands.png")).convert()
        self.screen.blit(self.img_manuals, (0, 0))
        pygame.display.flip()

        # Initialize aruco
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        print "Drone initialized"

    def start_main_loop(self):
        print "Main loop started"
        while self.loop_running:
            dt = time.time() - self.time
            if dt < 0.04:
                time.sleep(0.04 - dt)
            self.time = time.time()
            self.handle_key_stroke()
            if not self.use_webcam:
                self.update_video_from_drone()
            else:
                self.update_video_from_webcam()
            self.analyze_image()
            self.height = self.drone.navdata[0]['altitude']
            if self.automatic_mode:
                self.pid_controller(dt)
            self.movement_routine()
            self.print_intel()
            self.refresh_img(self.img, -90)

        self.drone.halt()

    def movement_routine(self):
        self.action = "Default"
        if self.automatic_mode:
            pass
        if abs(self.control["x"]) > +0.05 * (self.p_x + self.d_x):
            self.action = "3D rotation"
            self.drone.at(lib_drone.at_pcmd, True,
                          0,
                          +self.control["distance"],
                          -self.control["y"],
                          +self.control["x"])
        elif self.corners is not None or self.lag_counter > 0:
            self.action = "3D adjustment"
            self.drone.at(lib_drone.at_pcmd, True,
                    +self.control["x"],
                    +self.control["distance"],
                    -self.control["y"],
                    0)
        elif self.automatic_mode:
            self.action = "Hovering"
            self.drone.hover()

    def set_cycle_time(self, new_cycle_time):
        assert new_cycle_time > 0
        self.cycle_time = new_cycle_time

    def handle_key_stroke(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.loop_running = False
                self.drone.halt()
            elif event.type == pygame.KEYUP and not self.automatic_mode:
                self.drone.hover()
            elif event.type == pygame.KEYDOWN:
                if event.key not in [pygame.K_RIGHT, pygame.K_LEFT]:
                    self.turn = 0
                if event.key in [pygame.K_BACKSPACE, pygame.K_ESCAPE]:
                    self.drone.halt()
                    self.loop_running = False
                # takeoff / land
                elif event.key == pygame.K_RETURN:
                    print("Return pressed, taking off")
                    self.drone.takeoff()
                elif event.key == pygame.K_SPACE:
                    print("Space pressed, landing")
                    self.drone.land()
                elif event.key == pygame.K_r:
                    self.drone.reset()
                # activate program modes
                elif event.key == pygame.K_t:
                    self.automatic_mode = not self.automatic_mode
                    print "Automatic mode enabled:", self.automatic_mode
                elif self.automatic_mode:
                    continue
                # video
                elif event.key == pygame.K_v:
                    self.view_front_camera = not self.view_front_camera
                    self.drone.set_camera_view(self.view_front_camera)
                # forward / backward
                elif event.key == pygame.K_w:
                    print "Move forward"
                    self.drone.move_forward()
                elif event.key == pygame.K_s:
                    print "Move backward"
                    self.drone.move_backward()
                # left / right
                elif event.key == pygame.K_a:
                    print "Move left"
                    self.drone.move_left()
                elif event.key == pygame.K_d:
                    print "Move right"
                    self.drone.move_right()
                # up / down
                elif event.key == pygame.K_UP:
                    print "Move up"
                    self.drone.move_up()
                elif event.key == pygame.K_DOWN:
                    print "Move down"
                    self.drone.move_down()
                # turn left / turn right
                elif event.key == pygame.K_LEFT:
                    print "Turn left"
                    self.drone.turn_left()
                    self.turn = +1
                elif event.key == pygame.K_RIGHT:
                    print "Turn right"
                    self.drone.turn_right()
                    self.turn = -1
                # speed
                elif event.key == pygame.K_1:
                    self.drone.speed = 0.1
                elif event.key == pygame.K_2:
                    self.drone.speed = 0.2
                elif event.key == pygame.K_3:
                    self.drone.speed = 0.3
                elif event.key == pygame.K_4:
                    self.drone.speed = 0.4
                elif event.key == pygame.K_5:
                    self.drone.speed = 0.5
                elif event.key == pygame.K_6:
                    self.drone.speed = 0.6
                elif event.key == pygame.K_7:
                    self.drone.speed = 0.7
                elif event.key == pygame.K_8:
                    self.drone.speed = 0.8
                elif event.key == pygame.K_9:
                    self.drone.speed = 0.9
                elif event.key == pygame.K_0:
                    self.drone.speed = 1.0
        # After parsing the input events
        if self.turn > 0 and not self.automatic_mode:
            print "Continue rotating left"
            self.drone.turn_left()
        elif self.turn < 0 and not self.automatic_mode:
            print "Continue rotating right"
            self.drone.turn_right()

    def analyze_image(self):
        gray = cv.cvtColor(self.img, cv.COLOR_RGB2GRAY)
        all_corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if all_corners:
            self.corners = all_corners
            corners = all_corners[0][0]
            dim = corners.shape
            c = np.sum(corners, axis=0) / dim[0]
            self.center = (c[0], c[1])
            x1, y1 = corners[0][0], corners[0][1]
            x2, y2 = corners[1][0], corners[1][1]
            x3, y3 = corners[2][0], corners[2][1]
            x4, y4 = corners[3][0], corners[3][1]
            # A = (1 / 2) | [(x3 - x1)(y4 - y2) + (x4 - x2)(y1 - y3)] |
            self.marker_size = math.sqrt(abs((x3 - x1) * (y4 - y2) + (x4 - x2) * (y1 - y3)) / 2)
        else:
            self.corners = None
            self.center = (-1, -1)
            self.marker_size = 0

    def update_video_from_drone(self):
        self.img = self.drone.get_image()  # (360, 640, 3) or (720, 1280, 3)
        self.plot_analysis_result()

    def update_video_from_webcam(self):
        ret, self.img = self.cam.read()
        self.img = cv.cvtColor(self.img, cv.COLOR_BGR2RGB)
        self.plot_analysis_result()

    def refresh_img(self, array, rotate=0):
        surface = pygame.surfarray.make_surface(array)
        surface = pygame.transform.rotate(surface, rotate)
        surface = pygame.transform.flip(surface, True, False)
        self.screen.blit(surface, (0, 0))
        pygame.display.flip()

    def pid_controller(self, dt):
        err_height = self.height - self.ref_height
        self.integral["err_height"] += err_height / 40

        if self.corners is not None:
            err_x = self.center[0] / self.image_shape[1] - 0.5
            err_y = self.center[1] / self.image_shape[0] - 0.5
            err_distance = self.marker_size - self.ref_marker_size
            self.integral["err_x"] += err_x
            self.integral["err_y"] += err_y
            self.integral["err_distance"] += err_distance
            self.aruco_found = True
        # No aruco marker has been found
        elif self.lag_counter <= 2 and self.aruco_found:
            self.lag_counter += 1
            return
        # Also no aruco marker was found for some time
        else:
            self.lag_counter = 0
            self.aruco_found = False
            err_x = 0
            err_y = 0
            err_distance = 0
            self.integral["err_x"] *= 0.9
            self.integral["err_y"] *= 0.9
            self.integral["err_distance"] *= 0.9

        # Calculate the integral parts
        # Calculate new control values
        self.control["x"] = self.p_x * err_x \
                            + self.d_x * (err_x - self.last["err_x"])
                          # + self.K["D"] * (err_x - self.last["err_x"])
        self.control["y"] = self.p_y * err_y \
                            + self.d_y * (err_y - self.last["err_y"])
        self.control["height"] = 1 * err_height
                               # + self.K["I"] * self.integral["err_height"] \
                               # + self.K["D"] * (err_height - self.last["err_height"])
        self.control["distance"] = 1.0 / 1000.0 * err_distance
                                 # + self.K["I"] * self.integral["err_distance"] \
                                 # + self.K["D"] * (err_distance - self.last["err_distance"])
        # Save values for the next iteration
        self.last["err_x"] = err_x
        self.last["err_y"] = err_y
        self.last["err_height"] = err_height
        self.last["err_distance"] = err_distance

    def plot_analysis_result(self):
        if self.corners is not None:
            self.img = aruco.drawDetectedMarkers(self.img, self.corners)
            cv.circle(self.img, self.center, 2, (0, 0, 255), 2)

    def print_intel(self):
        font = cv.FONT_HERSHEY_SIMPLEX
        font_color = (0, 0, 0)
        font_size = 0.5
        font_weight = 2
        self.battery_level = self.drone.navdata.get(0, dict()).get('battery', 0)
        battery_text = "Battery level: {0:2.1f}%".format(self.battery_level)
        height_text = "Drone height: {0:d} mm".format(self.height)
        marker_size_text = "Marker size: {0:.1f} x10^3 px^2".format(self.marker_size / 1000)
        control_text_x = "dx = {0:.2f}".format(self.control["x"])
        control_text_y = "dy = {0:.2f}".format(self.control["y"])
        control_text_distance = "distance = {0:f}".format(self.control["distance"])
        cv.putText(self.img, battery_text, (5, 25), font, font_size, font_color, font_weight)
        cv.putText(self.img, height_text, (5, 55), font, font_size, font_color, font_weight)
        cv.putText(self.img, marker_size_text, (5, 85), font, font_size, font_color, font_weight)
        if self.automatic_mode:
            cv.putText(self.img, "AUTOMATIC MODE", (int(round(self.image_shape[1] * .4)), 30), font, 1, (0, 255, 0), 2)
            cv.putText(self.img, self.action, (int(round(self.image_shape[1] * .7)), 30), font, 1, font_color, 2)
            cv.putText(self.img, control_text_x, (int(round(self.image_shape[1] * .7)), 55), font, 1, font_color, 2)
            cv.putText(self.img, control_text_y, (int(round(self.image_shape[1] * .7)), 80), font, 1, font_color, 2)
            cv.putText(self.img, control_text_distance, (int(round(self.image_shape[1] * .7)), 105), font, 1, font_color, 2)



