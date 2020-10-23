#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry> 
#include <geometry_msgs/Wrench.h>

namespace my_simple_controllers {

class AdmittanceController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> 
{
    //std::vector<hardware_interface::JointStateHandle> jointStateHolder;
    std::vector<hardware_interface::JointHandle> velocityHolder;
    int numOfJoints;
    std::vector<double> Kp;
    KDL::Tree my_tree;
    KDL::Chain chain;
    ros::NodeHandle *nh;
    ros::Subscriber sub;
    Eigen::Matrix<double,6,1> input_force;
    Eigen::Matrix<double,6,1> prev_cart_pos;
    double curr_time;
    double start_time;
public:
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void control(const geometry_msgs::Wrench control);
    Eigen::Matrix<double,6,1> frameToEigen(KDL::Frame f)
    {
    Eigen::Matrix4d m;
    Eigen::Matrix3d mr;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            m(i,j)=f(i,j);
        }
    }
     for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            mr(i,j)=f(i,j);
        }
    }
    Eigen::Matrix<double,6,1> pos;
    Eigen::Vector3d eangc=mr.eulerAngles(0,1,2);
    pos(0,0)=m(0,3);
    pos(1,0)=m(1,3);
    pos(2,0)=m(2,3);
    pos(3,0)=eangc(0);
    pos(4,0)=eangc(1);
    pos(5,0)=eangc(2);  
    return pos;
    };
    Eigen::Matrix<double,3,6> DLSInv(Eigen::Matrix< double, 6, 3 > J , double k)
    {
        Eigen::Matrix<double,3,6> Jt=J.transpose();
        Eigen::Matrix<double,6,6> JJtInv=(J*Jt+k*k*Eigen::Matrix<double,6,6>::Identity()).inverse();
        return Jt*JJtInv;
    };
};
};

#endif
