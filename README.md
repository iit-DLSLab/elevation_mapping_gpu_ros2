# elevation_mapping_gpu_ros2
This repo contains the implementation of elevation mapping cupy for ros2 humble for the Nvidia Jetson Orin Board.

## Build Docker Image
In order to build the Docker image with all the required dependencies to run the elevation mapping cupy, run the following command. This will install a Ubuntu 22 environment with ROS2 Humble and CUDA 12 for ARM architecture machines.

Clone this repo:
```
git clone git@github.com:iit-DLSLab/elevation_mapping_gpu_ros2.git --b Orin
```
Build the Docker:
```
sudo docker build -f elevation_mapping_cupy/docker/Dockerfile.Orin -t elevation_mapping_cupy_ros2:latest .
```
# Run Elevation Mapping Package

## 1. Configure the Subscribers
In order to configure the input sensors for the package to work, you need to edit 3 files.
1. ```elevation_mapping_cupy/elevation_mapping_cupy/config/core/example_setup.yaml```
Here you specify the subscriber topic name and the data type (LIDAR and cameras)

2.  ```elevation_mapping_cupy/elevation_mapping_cupy/config/setups/menzi/base.yaml```
Here also you have to specify the subscriber topic name and data type, similar as in 1.

3. ```elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml```
Here you need to specify the: 
- map_frame
- robot_frame
- initialize_frame_id

To run the elevation mapping pipeline that are 2 ways, either with docker compose or with bash files.

## 2.A Docker Compose 
For easy executing commands, use the following docker compose commands to activate multiple services. The ros environment file can be found at ```elevation_mapping_cupy/docker/.ros-env```.
```
sudo docker compose -f elevation_mapping_cupy/docker/docker-compose.yaml up elevation_mapping -d
```

### 2.1A RVIZ Visualization


### 2.2A Docker compose debug



## 2.B Through Bash Files
### 2.1B Enter the Docker:
```
cd elevation_mapping_gpu_ros2
./elevation_mapping_cupy/docker/run.sh
```
**SIDE NOTE** </br> 
This repo was tested using **cyclonedds** as message protocol. If you are using the ZENOH protocol, you need to also run in another terminal inside the docker
```
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## 2.2B Launch the elevation mapping node
Once the subscribers are correctly configure you can run the elevation mapping package.
```
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py
```

If you want to use the code with a rosbag, then you have two options:
1. **Option 1:** Change the path [here](https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2/blob/main/elevation_mapping_cupy/docker/run.sh#L9), with the path where you rosbags are usually stored
2. **Option 2:** Run the Docker image with `./elevation_mapping_cupy/docker/run.sh /your_path/to/rosbags`

In both cases, when you enter the Docker you see two folders in your Docker home: `workspace` and `rosbags`. The first one is where you can access, possibly change (if needed), and build the code of the elevation mapping, where the second one is where you can run your rosbags with `ros2 run your_rosbag --loop`.

## 2.3B RVIZ Visualization
To visualize the grid_map on RViz:
	- Click `Add` --> `By display type` --> `GridMap`
	- Select `GridMap` --> then set the Topic to `/elevation_mapping_node/elevation_map_raw`
	- In the GridMap panel --> use the Layer dropdown to pick e.g. "elevation" or "traversability"
	You can do the same with the `/elevation_mapping_node/elevation_map_filter topic`

 ![image](https://github.com/user-attachments/assets/d5500a0c-a635-458d-a149-ed3debec73b6)


 




