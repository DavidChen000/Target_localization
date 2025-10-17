#ifndef GIMBAL_KINEMATICS_H
#define GIMBAL_KINEMATICS_H


#include <eigen3/Eigen/Dense>

//Gimbal's parameters uint:mm
#define HEIGHT 67
#define LENGTH 40
#define WIDTH1 20
#define WIDTH2 30




class GimbalKinematics
{
private:



    
public:
    GimbalKinematics(double theta,double d,double a,double alpha);

    //calculate the link transformational matrix
    Eigen::Matrix4d getLinkTransMatrix() const;

    //calculate the gimbal DH transformational matrix
    Eigen::Matrix4d getGimbalTransMatrix(const Eigen::Matrix4d &link1,const Eigen::Matrix4d &link2,const Eigen::Matrix4d &link3) ;

    double theta_;
    double d_;
    double a_;
    double alpha_;

    ~GimbalKinematics();
};

GimbalKinematics::GimbalKinematics(double theta,double d,double a,double alpha)
{
}

GimbalKinematics::~GimbalKinematics()
{
}




#endif //GIMBAL_KINEMATICS_H