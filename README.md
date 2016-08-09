# CDTM-Deep-Learning-Drones
Course Material for CDTM Deep Learning Drones Course

## Usage Docker Container
### Build from scratch using
```
nvidia-docker build -t YOURNICENAMEFORTHECONTAINER .
```
### Pull from Dockerhub.com
```
nvidia-docker pull patrickchrist/cdtm-deep-learning-drones
```
(or CPU-only:)
```
docker pull mjdev/cdtm-deep-learning-drones
```
### Start Docker Container Using
With Nvidia GPU:
```
sudo nvidia-docker run -v /home/$(whoami):/data -p 5000:5000 patrickchrist/cdtm-deep-learning-drones
```
or CPU-only:
```
docker run -v /home/$(whoami):/data -p 5000:5000 mjdev/cdtm-deep-learning-drones
```

### Enter a running Docker Container
Get the docker container id and remember it.
```
sudo nvidia-docker ps
```
Login using the following command.
```
sudo docker exec -it CONTAINERID bash
```
## Python ARDrone Lib
To install the Python ARDrone Lib for Windows follow please the README in /python-ardrone.

## Python ARDone Lib PS-Drone
Second Lib to control the drone.
## Object Detection Tutorial Using Digits DetectNET
### Download Data from Kitti
```
Labels
http://kitti.is.tue.mpg.de/kitti/data_object_label_2.zip
```
```
Data
http://kitti.is.tue.mpg.de/kitti/data_object_image_2.zip
```
```
Infos
http://kitti.is.tue.mpg.de/kitti/devkit_object.zip
```
### Follow Digits Tutorial
Follow the Digits Object Detection Tutorial.
[https://github.com/NVIDIA/DIGITS/tree/master/examples/object-detection]


