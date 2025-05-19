# elevation_mapping_gpu_ros2
This repo contains the implementation of elevation mapping cupy for ros2 humble.
Issues and planning related to the elevation mapping. We can share the code, issues, etc.. here


## Build Docker Image
In order to build the Docker image with all the required dependencies to run the elevation mapping cupy, run the following command. This will install a Ubuntu 22 environment with ROS2 Humble and CUDA 12.1 for x64 architecture machines.

Clone this repo:
```
git clone git@github.com:iit-DLSLab/elevation_mapping_gpu_ros2.git
```
Build the Docker:
```
sudo docker build -f elevation_mapping_cupy/docker/Dockerfile.x64 -t elevation_mapping_cupy_ros2:latest .
```
Enter the Docker:
```
cd elevation_mapping_gpu_ros2
./elevation_mapping_cupy/docker/run.sh
```

To launch the elevation map:
1) In Terminal 1 (T1):
 ```
 ros2 run rmw_zenoh_cpp rmw_zenohd
 ```
2) Find the container name with docker ps, then in T2:
```
docker exec -it <container-name-or-id> bash
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py
```

If you want to use the code with a rosbag, then you have two options:
1. **Option 1:** Change the path [here](https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2/blob/main/elevation_mapping_cupy/docker/run.sh#L9), with the path where you rosbags are usually stored
2. **Option 2:** Run the Docker image with `./elevation_mapping_cupy/docker/run.sh /your_path/to/rosbags`

In both cases, when you enter the Docker you see two folders in your Docker home: `workspace` and `rosbags`. The first one is where you can access, possibly change (if needed) and build the code of the elevation mapping, where the second one is where you can run your rosbags with `ros2 run your_rosbag --loop`.



