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

'''
sudo nvidia-docker run -v /home/YOURACCOUNT:/data  -p 5000:5000 patrickchrist/cuda-digits-resnet
'''
