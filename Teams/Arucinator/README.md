# Drone Elective at the _Center for Digital Technology and Management_ (CDTM)
## Contributors
* [Daisy Paiva](mailto:dsouzapaiva@gmail.com)
* [Martin Patz](mailto:mailto@martin-patz.de)
 
## Approach
Our main approach was to use [Aruco markers](http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html) and a [PID](https://en.wikipedia.org/wiki/PID_controller) controller to control the Drone.
For that we used the [libardrone](https://github.com/venthur/python-ardrone/blob/master/libardrone.py) library.
The Aruco markers are detected with the _opencv_ library.

## Outcome
We delivered a quite good race.
Eventually we finished in place 5 out of 9.
Our main issue were connection problems between the Drone and the Notebook, running the python script.
This was especially severe in the _TUM Maschinenbau_ building, because many people, many phones, and many Drones (each hosting its own WiFi hotspot).
