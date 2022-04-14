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

![](/lidar-curb.gif)
