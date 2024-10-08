# Use an official Ubuntu 20.04 image as the base
FROM ubuntu:20.04

# Set environment variable to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies, including tools and libraries
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    libsfml-dev \
    libgtest-dev \
    make \
    libspdlog-dev \      # Adding spdlog
    libfmt-dev \         # Adding fmt
    git \                # Adding git to clone and install libraries that are not available via apt
    && rm -rf /var/lib/apt/lists/*

# Install Google Test manually (since gtest is only source code in libgtest-dev)
RUN cd /usr/src/gtest && \
    cmake . && \
    make && \
    cp lib/*.a /usr/lib

# Install Prometheus C++ libraries (example installation, adjust as necessary)
RUN git clone https://github.com/jupp0r/prometheus-cpp.git && \
    cd prometheus-cpp && \
    git submodule update --init && \
    mkdir _build && \
    cd _build && \
    cmake .. -DBUILD_SHARED_LIBS=ON && \
    make && make install

# Set the working directory inside the container
WORKDIR /app

# Copy the contents of the project directory from the host to the container
COPY . /app

# Create a build directory, configure the project with CMake, and compile
RUN mkdir build && cd build && cmake .. && make

# Specify the default command to run the project executable
CMD ["./build/rrt_3d"]
