# Start with cuDNN base image
FROM nvidia/cuda:7.5-cudnn5-devel

# Install git, wget, bc, cmake and dependencies
RUN apt-get update && apt-get install -y \
  git \
  wget \
  bc \
  cmake \
  libgflags-dev \
  libgoogle-glog-dev \
  libopencv-dev \
  libleveldb-dev \
  libsnappy-dev \
  liblmdb-dev \
  libhdf5-serial-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libatlas-base-dev \
  python-dev \
  python-pip \
  python-numpy \
  python-opencv \
  gfortran
# Install boost
RUN apt-get install -y --no-install-recommends libboost-all-dev

# Clone NVIDIA Caffe repo and move into it
RUN cd /root && git clone -b caffe-0.15 https://github.com/NVIDIA/caffe.git && cd caffe && \
# Install python dependencies
  cat python/requirements.txt | xargs -n1 pip install
  

# Move into NVIDIA Caffe repo
RUN cd /root/caffe && \
# Make and move into build directory
  mkdir build && cd build && \
# CMake
  cmake -DCPU_ONLY=ON .. && \
# Make
  make -j"$(nproc)"
# Set CAFFE_HOME
ENV CAFFE_HOME /root/caffe

# Clone DIGITS repo and move into it
RUN cd /root && git clone -b digits-4.0 https://github.com/NVIDIA/DIGITS.git digits && cd digits && \
# pip install
  pip install -r requirements.txt

# Expose server port
EXPOSE 5000
# Start server
CMD ["/root/digits/digits-devserver"]


  
  
