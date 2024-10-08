# Use an official Ubuntu 20.04 image as the base
FROM ubuntu:20.04

# Set environment variable to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies: CMake, g++, SFML, Google Test, and any necessary libraries
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    libsfml-dev \
    libgtest-dev \
    make \
    && rm -rf /var/lib/apt/lists/*

# Install Google Test manually (since gtest is only source code in libgtest-dev)
RUN cd /usr/src/gtest && \
    cmake . && \
    make && \
    cp lib/*.a /usr/lib

# Set the working directory inside the container
WORKDIR /app

# Copy the contents of the project directory from the host to the container
COPY . .

# Create a build directory, configure the project with CMake, and compile
RUN rm -rf build && mkdir build && cd build && cmake .. && make

# Specify the default command to run the project executable (replace this with your desired executable)
CMD ["./build/rrt_3d"]
