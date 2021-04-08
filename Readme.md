# ROS2 Latency Experiments
Package to compare pointcloud (PC) publishing latency using `rclypp` or `rclpy`.

### Nodes
* `source` (C++)  
  Publishes dummy pointcloud (PC) data of a specific size
* `repeater` (C++)  
  Copies the incoming PC and re-publishes on another topic
* `repeater.py` (Python)  
  Same as the C++ repeater, but using rclpy. A significant latency on the pub.publish(...) call is observed.
* `measure` (C++)
  Measures the age of an incoming PC by comparing the stamp with the current ROS time. This is to double-check if the latency is actually observed on the receiving end.

### Launch Files
* `experiment.launch.py`  
  Launches all nodes. Arguments:  
  * `megabytes` (int)
    The size of the created dummy pointcloud, to compare different sizes.

## Observations
On a fast desktop system with:
* Ryzen 3800X CPU
* 16GB RAM
* SSD

#### 1 MB Pointcloud
(like from a LIDAR sensor)
````
>> ros2 launch ros2_latency experiment.launch.py megabytes:=1
[source-1]      [INFO] [1617894568.132222845] [ros2_latency]: creating   took 3898 [us]
[source-1]      [INFO] [1617894568.132272495] [ros2_latency]: publishing took 118 [us]
[repeater-2]    [INFO] [1617894568.133056859] [ros2_latency]: copying    took 121 [us]
[repeater-2]    [INFO] [1617894568.133102729] [ros2_latency]: publishing took 116 [us]
[measure-4]     [INFO] [1617894568.133555481] [ros2_latency]: age of pointcloud via cpp is 611.183 [us]
[repeater.py-3] [INFO] [1617894568.145218816] [ros2_latency]: Age of source PC is  1346 [us]
[repeater.py-3] [INFO] [1617894568.145447607] [ros2_latency]: Copying took    92 [us]
[repeater.py-3] [INFO] [1617894568.145639368] [ros2_latency]: Publishing took 11310 [us]
[measure-4]     [INFO] [1617894568.145463717] [ros2_latency]: age of pointcloud via py  is 13350 [us]
````
*-> Publishing the PC via Python takes 13ms, via C++ only 0.11ms*

#### 10 MB Pointcloud
(like from a 640x480 RGBD camera)
````
[source-1]      [INFO] [1617894523.025022662] [ros2_latency]: creating   took 28423 [us]
[source-1]      [INFO] [1617894523.025084873] [ros2_latency]: publishing took 1406 [us]
[repeater-2]    [INFO] [1617894523.035501434] [ros2_latency]: copying    took 2006 [us]
[repeater-2]    [INFO] [1617894523.035576994] [ros2_latency]: publishing took 2321 [us]
[measure-4]     [INFO] [1617894523.041298005] [ros2_latency]: age of pointcloud via cpp is 8112.71 [us]
[repeater.py-3] [INFO] [1617894523.133180090] [ros2_latency]: Age of source PC is 11477 [us]
[repeater.py-3] [INFO] [1617894523.133437451] [ros2_latency]: Copying took  1961 [us]
[repeater.py-3] [INFO] [1617894523.133638792] [ros2_latency]: Publishing took 95739 [us]
[measure-4]     [INFO] [1617894523.137496766] [ros2_latency]: age of pointcloud via py  is 113877 [us]
````
*-> Publishing the PC via Python takes 114ms, via C++ only 2.3ms*
