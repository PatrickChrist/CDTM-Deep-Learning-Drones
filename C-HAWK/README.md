# C-HAWK

This drone control mainly uses the libraries OpenCV and [libardrone](https://github.com/venthur/python-ardrone).
The drone will follow a chessboard. For this function the [chessboard recognition](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html) from OpenCV is used.
After the drone has detected the chessboard, it computes the middle of the board and the length of the diagonal of the board.
These informations are put into three different PD-Controller ([PID-Controller](https://en.wikipedia.org/wiki/PID_controller) with the I-constant equal to 0).
Thereby three "directions" are optimised:
* x-coordinate -> left/right-control
* y-coordinate -> height-control
* length of diagonale -> backwards/forwards-control

## To run the program:
You have to download the GitHub project [libardrone](https://github.com/venthur/python-ardrone).
Then put the files from this project into the libardrone folder.
Connect your laptop to the drone and run CentralControl.py.
For takeoff press any key. 
You should be able to see the front camera livestream and if the drone detects a chessboard, you will see a marking in the livestream.
Now the drone should try to follow the chessboard.
For landing and shutting down the drone press space.

## Overview over the included files:
* CentralControl.py:
    + the drone is started
    + values of the patternRecognition are received, interpreted and given to the three PD-Controllers
    + values of the PD-Controllers are received, interpreted and used to calculate the speeds and directions for actuating the desired movements
    + the drone is landed and shut down
* PIDController.py: a normal implementation of a PID-Controller where the constants are set via parameters
* Testprotocol.txt: some example values for speed settings which worked well
* patternRecognition.py:
    + function that detects a (n x m) chessboard in an image and returns the coordinates of the left upper corner and the right bottom corner
    + the function uses 'findChessboardCorners' of the openCV library to find the chessboard
    + the chessboard is marked in the original image and is shown in a seperate window
* schachbrettmuster.jpg: example chessboard (the code expects this chessboard size, but you can configure patternRecognition.py if you want to use another chessboard size)
* schachmuster_5x5.jpg: example chessboard

**_Authors:_** Christian Gebhardt and Christian Münch
