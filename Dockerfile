# Use the official ROS Melodic base image
FROM osrf/ros:melodic-desktop-full

# Set the working directory
WORKDIR /workspace

# Install additional dependencies if needed
# For example, you can uncomment the line below to install a package
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    libxml2-dev \
    libxslt-dev \
    python \
    python-pip\
    libarmadillo-dev\
    wget

# Download, extract, and build nlopt
RUN git clone -b v2.7.1 https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig


# Download, extract, and build LKH
RUN wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz && \
    tar xvfz LKH-3.0.6.tgz && \
    cd LKH-3.0.6 && \
    make && \
    cp LKH /usr/local/bin && \
    cd .. && \
    rm -rf LKH-3.0.6 LKH-3.0.6.tgz

COPY . /workspace/src/

WORKDIR /workspace

RUN . /opt/ros/melodic/setup.sh && catkin_make

# Set entry point to start ROS
CMD ["bash"]
