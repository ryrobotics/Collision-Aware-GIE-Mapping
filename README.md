

# Collision-Aware-GIE-Mapping

Mapping-related source code for the paper: **Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation**. For the other modules of Air Bumper, please refer to [this repo](https://github.com/ryrobotics/air_bumper).

This work has been accepted by 2024 IEEE International Conference on Robotics and Automation (ICRA).

This software is a collision-aware volumetric mapping system, which effectively calculates Occupancy Grid Maps (OGMs) and Euclidean Distance Transforms (EDTs) with GPU. The proposed system achieves real-time performance with limited onboard computational resources.

The supplementary video can be viewed here:

<p align="center">
  <a href="https://youtu.be/FVQGmqUTyp4" target="_blank"><img src="https://github.com/ryrobotics/ryrobotics.github.io/blob/main/content/publication/wang-2023-air/featured.png" alt="video" width="800" height="450" border="1" /></a>
</p>

Please cite at least one of our papers if you use this project in your research:

```
@misc{wang2023air,
    title={Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation}, 
    author={Ruoyu Wang and Zixuan Guo and Yizhou Chen and Xinyi Wang and Ben M. Chen},
    year={2023},
    eprint={2307.06101},
    archivePrefix={arXiv},
    primaryClass={cs.RO}}
```
```
@article{chen2022gie,
    author={Chen, Yizhou and Lai, Shupeng and Cui, Jinqiang and Wang, Biao and Chen, Ben M.},
    journal={IEEE Robotics and Automation Letters}, 
    title={GPU-Accelerated Incremental Euclidean Distance Transform for Online Motion Planning of Mobile Robots}, 
    year={2022},
    volume={7},
    number={3},
    pages={6894-6901},
    doi={10.1109/LRA.2022.3177852}}
 ````

## 0. Supported data input:
- Any sensor outputs pointcloud
- Depth camera (Intel D435i)
- 2D LiDAR (RPLiDAR S1)
- 3D LiDAR (Livox Mid360, Ouster OS0, Velodyne VLP-16)
- Priori knowledge


## 1. Installation

### Prerequisite

1. This project runs CUDA and requires a computer with **Nvidia GPU**. We have successfully tested this project on CUDA 9.0, 10.2, 11.3, 11.4, 11.8 and 12.0.
2. Install Ubuntu with ROS. This project has been tested on Ubuntu 16.04 (ROS Kinetic), 18.04 (ROS Melodic) and 20.04 (ROS Noetic). 
### Recompile cuTT 
cuTT is a library used for faster batch EDT. 
````bash
git clone https://gitee.com/jinxer000/cutt_lts.git
cd cutt_lts
rm ./build/*
make 
````
It will create  the library itself:
- include/cutt.h
- lib/libcutt.a
 
Copy the lib file (lib/libcutt.a) into $(GIE_folder)/lib.

If it fails to compile, please modify the Makefile according to [this website](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/).

### Seperate compilation lib
Search your computer and find the *libcudadevrt.a* (e.g., /usr/local/cuda-10.2/targets/x86_64-linux/lib/libcudadevrt.a).

Copy the lib file (libcudadevrt.a) into $(GIE_folder)/lib.

### Compile 
Download source code from github:
```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ryrobotics/Collision-Aware-GIE-Mapping.git
cd ..
catkin build
source devel/setup.bash
```

### Trick
Here are some pre-built lib files in lib folder, you can rename it as mentioned above and try to compile.

If the compilation with pre-built files failed, you should follow the steps carefully and retry it.

Please kindly leave a star :star: if this software is helpful to your projects : )

## 2. Try on your own robot
Simply remap the input data in volumetric_mapper.cpp to your own sensor topics!

Remember to set *use_sim_time* parameter in each launch file as **false** in the real world.

### Speed up tricks
- Turn off Rviz during the run since it will occupy a large amount of GPU resources.

- Disable both *display_glb_edt* and *display_glb_ogm* parameter. Hence the GPU hash table won't be streamed to CPU at every iteration.

- Decrease the parameter *cutoff_dist* to a small number (e.g., 2m).

- Turn on *fast_mode* parameter. It will disable wavefront A and wavefront B (please see details in our [paper](https://ieeexplore.ieee.org/abstract/document/9782137)). If working in confined space, (e.g., Cow-Lady dataset), the accuracy is nearly the same as the original mode.

### Caution
- If the local size is too large, an "invalid argument" error will be thrown due to CUDA does not support such a large thread-block.

- The parameters *bucket_max* and *block_max* has to be increased if you are doing large-scale and fine-resolution mapping. The initialization time may be longer. 

### Integrate with motion planners
Please set *for_motion_planner* parameter as true. It makes the current robot position valid and observed.

Our system publishes the EDT surround by the robot as CostMap.msg in the topic "cost_map". Each voxel contains visibility information and the distance value. If your motion planning package are not implemented together with GIE, then you can only access the local EDT information by subscribing to the topic "cost_map".

To access the global EDT directly, you are recommended to implement a  GPU-based motion planner together with GIE-mapping. 
Each voxel  can be retrieved by using device function  *get_VB_key()* and *get_voxID_in_VB()*. In this way, the *display_glb_ogm* parameter can be *false*, saveing you tons of time.

If you are using a  CPU-based planner, you can retrieve the voxel block ID like this:
```cpp
 int VB_idx =_hash_map->hash_table_H_std.find(blk_key)->second;
```
And voxels inside the block can be visited with *get_voxID_in_VB()*.

## 3. Additional features
### Frontiers for exploration
The system extracts low-level frontiers for exploration. The data type of **VOXTYPE_FNT** denotes the voxel that belongs to the low-level frontier. You may need to do some post-process to filter out the noise. 

### Signed distance 
Developing

### Virtual fence 
Revise the prebuilt map in ext_obsv.yaml. Note that pre_obs_bbx_ll and pre_obs_bbx_ur are the lower-left corner and upper-right corner of the flyable region. 

### External observer:
Publish a pointcloud in topic "forbid_reg_cloud".
If the pointcloud is 3D, please set **is_ext_obsv_3D** as true. Otherwise, the height of external observed obstacle is 0.2m~2.6m.

## 4. Trouble shooting
As reported in [issue 1](https://github.com/JINXER000/GIE-mapping/issues/1), there might be some problems in launching the mapper with Ubuntu 20.04. Please ensure that the GPU model, GPU driver version, and CUDA version match with each other. For more details, you can refer to [this website](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/). And it is recommend to activate C++ 14 in CMake when using 20.04.

Furthermore, this software is developed based on [GIE-mapping](https://github.com/JINXER000/GIE-mapping), we recommend you to visit the original repo for more information and datasets to test the mapping system.