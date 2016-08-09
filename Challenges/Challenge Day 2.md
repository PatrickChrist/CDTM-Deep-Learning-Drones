# Challenge Day 2
## 1. Connect to the GPU Lab
Connect to the GPU Lab using ssh and open Digits Webserver at localhost:5000
```
ssh -L 5000:localhost:5000 gu95wot@hpcXX.clients.eikon.tum.de
```
Where XX is your team number. Follow the installation guide in GPU-Lab-Tutorial.
## 2. Run the Digits Tutorial LeNet
Run the Digits LeNet Tutorial. https://github.com/NVIDIA/DIGITS/blob/master/docs/GettingStarted.md
## 3. Classify using python
Use the trained model from 2 and classify an image from your computer using python.
## 4. Run the Digits Object Detection Tutorial
Run the Object detection tutorial. https://github.com/NVIDIA/DIGITS/tree/master/examples/object-detection
## 5. Brainstorm about your classifier
Brainstorm what you want to track or recognize. Look for datasets online or generate them yourself. Discuss your ideas with the coaches and start hacking.
## Further resources
- https://github.com/NVIDIA/DIGITS/tree/master/examples/fine-tuning
- https://github.com/BVLC/caffe/blob/master/examples/net_surgery.ipynb
- https://docs.google.com/presentation/d/1UeKXVgRvvxg9OUdh_UiC5G71UMscNPlvArsWER41PsU/edit#slide=id.p
- https://github.com/BVLC/caffe/wiki/Model-Zoo
- https://github.com/BVLC/caffe/blob/master/examples/00-classification.ipynb
