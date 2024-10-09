### Project Dependencies 
Use Original documentation of the dependencies to install the libraries
```
GoogleTest                               //unit tests
SFML                                     //visualization library
C++ 17, G++ Compiler
fmt                                       //for logging
spdlog                                    //for logging
minikube                                  //scaling and container management
```

### 2D Path Planning Algorithm
You can run it directly using the executable, if you want to build it again locally, follow these steps 
```
cd RRT_Project/build
cmake ..
make

//once it is built, you can run the executable
./rrt_3d
```
### Visualization
visualization for RRT Path Planning is done using SFML library for 2D, you can expand it to 3D if needed, since path planning is for mobile robot 2D path planning is enough. Here is an example output: 
![Screenshot from 2024-10-07 19-29-36](https://github.com/user-attachments/assets/9d7d8b5c-7713-42d1-8e59-5bd7c095022c)

### Containerization Using Docker
Docker was used for containerization, you can change the dockerfile as per your dependency requirements 

To build the docker image of the project,use the command 
```
docker build -t rrt_project .
```
![Screenshot from 2024-10-08 13-59-15](https://github.com/user-attachments/assets/cee03a7a-6cbc-4bb3-a51a-03ca972d5910)

You can run the image directly, but it would be a single run
```
docker run -it rrt_project
```
### Scaling Of Tests Using Kubernetes
Kubernetes (minikube) is being used for local container management and then logs are being attained by using shell script, you would need to create a .yaml file for specifying kubernetes jobs details of your pods in the cluster. 
Please look at the rrt_project_jobs.yaml file for viewing job parallelism and other detials,commands for running minikube are: 
```
minikube start
```
![Screenshot from 2024-10-08 14-19-49](https://github.com/user-attachments/assets/4301bcf4-d0ae-4e18-bfc8-5de72091779c)
Set environment as the local minikube environment: 
```
eval $(minikube -p minikube docker-env)
```
To deploy the pods as kubernetes jobs:
```
kubectl apply -f rrt_project_jobs.yaml
```
Check the status of the pods using the command, use fetch_all_logs.sh shell script to download all logs to analyze runtimes.
```
kubectl get jobs
```
Retrieve logs:
```
chmod +x fetch_all_logs.sh
./fetch_all_logs.sh
```
**10 logs are uploaded in the repo for reference**

