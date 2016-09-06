# EAGLE-EYE

EAGLE-EYE was developed during the [CDTM Deep Learning Drones Elective](https://github.com/PatrickChrist/CDTM-Deep-Learning-Drones) and allows to control the AR.Drone 2. The drone can be remote controlled using the camera stream and the keyboard of the PC or switched into autonomous flight mode where it follows a 6x6 [ArUco](http://www.uco.es/investiga/grupos/ava/node/26) marker.

This project combines the features of a low level control library [libardrone](https://github.com/mjdev/python-ardrone) with the marker tracking functionalities of [OpenCV 3.1](http://opencv.org/opencv-3-1.html) and a software [PID controller](https://en.wikipedia.org/wiki/PID_controller) to achieve autonomous behavior.
This drone control mainly uses the libraries OpenCV and [libardrone](https://github.com/venthur/python-ardrone).
Once in autonomous mode, the drone follows the biggest visible ArUco marker. The controller determines it's steering commands the following way:
* Area of the marker: forward/backwards flight
* x-Axis offset to the frame's center: turn left/right
* y-Axis offset to the frame's center: move up/down
Suggested improvements:
* add left/right movement without turning
* optimize the controller values of the PID controller

## Setup and Run EAGLE-EYE:
The code was developed for the AR.Drone 2 using Python 2.7 and OpenCV 3.1.0.
The GitHub project [libardrone](https://github.com/mjdev/python-ardrone) provides a set of low level control functions that are used to control the drone. This library must be installed before being able to execute EAGLE-EYE.
If the libardrone library is contained in your python environment, the EAGLE-EYE repository can be cloned into a new directory and started by executing eagle_eye.py.
Make sure that you are connected to the drone's WiFi!

Press the return key to take off and control the drone using the following keys:
* UP - fly higher
* DOWN -fly lower
* LEFT - turn left
* RIGHT - turn right
* W - move forward
* S - move backwards
* A - move to the left
* D - move to the right
* M - toggle between manual and autonomous control
* SPACE - landing and shutdown

Note: Depending on your setup you might have to adjust the keycodes for the arrow keys since those might be different to the ones used on our machines.

While in autonomous mode, the drone will track and follow the biggest visible 6x6 ArUco marker.

**_Authors:_** Hagen Schmidtchen and Isabel Poppek

