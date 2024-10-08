### Project Dependencies 
Use Original documentation of the dependencies to install the libraries
```
GoogleTest //unit tests
SFML //visualization library
C++ 17, G++ Compiler
fmt //for logging
spdlog //for logging
prometheus // for custome metric generation
```

### 2D Path Planning algorithm
You can run it directly using the executable, if you want to build it again locally, follow these steps 
```
cd RRT_Project/build
cmake ..
make

//once it is built, you can run the executable
./rrt_3d
```
### Visualization
visualization is done using SFML library for 2D, you can expand it to 3D if needed, since path planning is for mobile robot 2D path planning is enough. Here is an example output: 
![Screenshot from 2024-10-07 19-29-36](https://github.com/user-attachments/assets/9d7d8b5c-7713-42d1-8e59-5bd7c095022c)

### Containerization and Scaling of Tests 
Docker was used for containerization, Kubernetes is being used for container management and then custom metrics are being attained by using Prometheus 
