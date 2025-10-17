#include "gimbal_viservo/gimbal_viservo_controller.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"gimbal_viservo_controller_node");

    ros::NodeHandle nh;
    double yawP,yawI,yawD,pitchP,pitchI,pitchD,integralLimitX,integralLimitY;

    nh.param("gimbal_viservo_controller_node/yawP_",yawP,0.08);
    nh.param("gimbal_viservo_controller_node/yawI_",yawI,0.00005);
    nh.param("gimbal_viservo_controller_node/yawD_",yawD,0.01);
    nh.param("gimbal_viservo_controller_node/pitchP_",pitchP,0.08);
    nh.param("gimbal_viservo_controller_node/pitchI_",pitchI,0.00005);
    nh.param("gimbal_viservo_controller_node/pitchD_",pitchD,0.01);
    nh.param("gimbal_viservo_controller_node/integralLimitX_",integralLimitX,200.0);
    nh.param("gimbal_viservo_controller_node/integralLimitY_",integralLimitY,200.0);
    


    gimPIDController gimbal_viservo_node(nh,yawP,yawI,yawD,pitchP,pitchI,pitchD,integralLimitX,integralLimitY);

    ros::spin();
    

    return 0;


}

 