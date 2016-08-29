C-HAWK

This drone control mainly uses the libraries OpenCV and libardrone.
The drone will follow a chessboard. For this function the chessboard recognition from OpenCV is used.
After the drone has detected the chessboard, it computes the middlw of the board and the length of the diagonal of the board.
These informations are put into three different PD-Controler.
Thereby three "directions" are optimised:
  x-coordinate -> left/right-control
  y-coordinate -> height-control
  length of diagonale -> backwards/forwards-control

To run the program:
You have to download the GitHub project brandneb/python-ardrone.
Then put the files from this project into the libardrone folder.
Connect your laptop to the drone and run CentralControl.py.
For takeoff press any key. 
Now the drone should try to follow the chessboard.
For landing the drone press space.
