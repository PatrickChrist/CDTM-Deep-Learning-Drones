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
### Start Docker Container Using
```
sudo nvidia-docker run -v /home/YOURACCOUNT:/data  -p 5000:5000 patrickchrist/cuda-digits-resnet
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


