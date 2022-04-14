# LiDAR-CURB-DETECTION

Simple algorithm to detect the curb of road environment using 3D LiDAR <br/>

<br/>

## Tested Environment

- Ubuntu 18.04
- ROS Melodic

<br/>

## Summary

* Subscribe `sensor_msgs/PointCloud2` msg
* Curb Detection from the point cloud
* Publish seperate curb (e.g. **left_curb**, **right_curb**)
  * left_curb : the red points in below figure
  * right_curb : the green points in below figure

![](/lidar-curb.gif)

<br/>

## How to use

Clone, build and run

~~~bash
$ git clone https://github.com/bigbigpark/LiDAR-CURB-DETECTION.git
~~~

~~~bash
$ catkin build
~~~

~~~bash
$ roslaunch lidar_curb_detection curb_detect.launch
~~~

<br/>

## Parameter configuration

You can easily modify topic name of `sensor_msgs/PointCloud2` by changing **curb_detect.launch** <br/>

~~~xml
<node name="curb_detection" pkg="lidar_curb_detection" type="curb_detection" respawn="true" output="screen">
    <remap from="/input" to="/os_cloud_node/points"/>
</node>
~~~

Here, change your topic <br/>

<br/>

## TODO

- [ ] Robust outlier removal
