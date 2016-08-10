# Set the following environment variables in your .bashrc (copy and paste

following lines in between ```)

```

###########################################################

# COMMON

###########################################################

# CUDA

###########################################################

export CUDA_BIN_PATH=/usr/local/cuda/bin

export CUDA_INC_PATH=/usr/local/cudasdk/C/common/inc

export CUDA_LIB_PATH=/usr/local/cuda/lib64

export CUDA_INSTALL_PATH=/usr/local/cuda

export LD_LIBRARY_PATH=/usr/local/cuda/lib:${LD_LIBRARY_PATH}

export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

###########################################################

```

# Download Anaconda

```
wget https://repo.continuum.io/archive/Anaconda2-4.0.0-Linux-x86_64.sh
```

# install anaconda locally (Say yes to all defaults)

```
bash Anaconda2-4.0.0-Linux-x86_64.sh
```

# create a local environment by cloning the default

```
export PATH=/home/$(whoami)/anaconda2/bin:$PATH

conda create -- name deeplearn -- clone root

source activate deeplearn
```

# Install Caffe

```
git clone -b caffe-0.15 https://github.com/NVIDIA/caffe.git && cd caffe
```

# Install python dependencies

```
cat python/requirements.txt | xargs -n1 pip install
```


# Make and move into build directory

```
mkdir build && cd build
```

# CMake

```
cmake ..
```

# Make

```
make -j"$(nproc)"
```

# CLone Digits

```
cd ..

git clone -b digits-4.0 https://github.com/NVIDIA/DIGITS.git digits && cd digits
```

# pip install

```
pip install -r requirements.txt
```

# Open again the .bashrc and add the following line Adjust the path to your caffe folder

```
export CAFFE_ROOT=/home/$(whoami)/Downloads/digits-4.0/caffe
```
(Adapt Path accordingly)

# Restart your terminal

# Important every time you restart your terminal you have to activate the

correct anaconda environment i.e. every time you open a terminal type:

source activate deeplearn

# run digits

```
cd caffe/digits

./digits-devserver
```

# open digits in the web browser

Type in the followin URL: [localhost:5000](localhost:5000)



# start the digits tutorial by opening the following URL

https://github.com/NVIDIA/DIGITS/blob/master/docs/GettingStarted.md
