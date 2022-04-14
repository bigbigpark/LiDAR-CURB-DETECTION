/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-14 19:31
 */
#include <lidar_curb_detection/curb_detection.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "curb_detection");
    ros::NodeHandle nh;

    CurbDetector CD;
    
    ros::spin();
    
    return 0;
}