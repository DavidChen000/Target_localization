#include <ros/ros.h>
#include "gimbal_viservo/gimbal_orientation_track.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"gimbal_viservo_node");

    ros::NodeHandle nh;
    
    gimbalOrienTrack gimbalOrienTrack(nh);

    ros::spin();

    return 0;
}