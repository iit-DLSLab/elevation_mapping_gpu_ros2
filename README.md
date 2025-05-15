# elevation_mapping_ros2
This repo contains the implementation of elevation mapping cupy for ros2 humble. 
Issues and planning related to the elevation mapping. We can share the code, issues, etc.. here


## Build Docker Image
In order to build the docker image with all the required dependencies to run the elevation mapping cupy, run the following command. This will install a Ubuntu 22 environment with ros2 humble and cuda12.1 for x64 architecture machines.

```
sudo docker build -f elevation_mapping_cupy/docker/Dockerfile.x64 -t elevation_mapping_cupy_ros2:latest .
```


