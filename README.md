# vslam

## ORB_SLAM2 ROS/catkin

* PureLocalization
* build map
### INSTALL
#### Depends
* DBoW2
``` shell
cd orb_slam2_ros/Thirdparty/DBoW2
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```
*  g2o
``` shell
cd g2o/Thirdparty/g2o
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```
### Edit launch file
``` launch
 <param name = "PureLocalization" type ="bool" value = "true" />
```
### RUN
```shell
  roslaunch orb_slam2_ros orb_slam2.launch
```
