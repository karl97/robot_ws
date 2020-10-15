#include "ros/ros.h"
#include <Eigen/Geometry> 
#include <geometry_msgs/Twist.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <ctime>
#include <cstdlib>
#include <sensor_msgs/JointState.h>


class kdl_test
{
    KDL::Tree my_tree;
    KDL::Chain chain;
    ros::NodeHandle *nh;
    int chainlength;
    KDL::JntArray joint_pos;
    ros::Subscriber sub;
public:
    kdl_test(ros::NodeHandle *n,std::string base_link,std::string tip_link)//sets up the chain
    {
        nh=n;
        sub = n->subscribe("joint_states", 1000, &kdl_test::js,this); 
        std::string robot_desc_string;
        nh->param("robot_description", robot_desc_string, std::string());
        if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
        {
            ROS_INFO("Failed to construct kdl tree");     
        }
        ROS_INFO("succeded to construct kdl tree"); 

        if (!my_tree.getChain(base_link, tip_link, chain))
        {
            ROS_INFO("Couldn't find chain from %s to %s\n", base_link.c_str(), tip_link.c_str());
        }
        else
        {
            ROS_INFO("succeded to find chain from %s to %s\n", base_link.c_str(), tip_link.c_str());
        }
            chainlength=chain.getNrOfJoints();
    };
    
    KDL::Frame solveForwardKinematics()
    {
        KDL::Frame cart_pos;
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        fk_solver.JntToCart(joint_pos, cart_pos);
        return cart_pos;
    };

    Eigen::Affine3d KDL_to_Eigen(KDL::Frame f)
    {
        Eigen::Matrix4d m;
        Eigen::Affine3d t;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                m(i,j)=f(i,j);
            }
        }
        t.matrix()=m;
        return t;
    };

    Eigen::Matrix<double,6,1> getVelocity(Eigen::Affine3d tcurrent, double dt)
    {
        
        Eigen::Matrix<double,6,1> v;
        v(0,0)=tcurrent.translation().x();
        v(1,0)=tcurrent.translation().y();
        v(2,0)=tcurrent.translation().z();
        Eigen::Vector3d eangc=tcurrent.rotation().eulerAngles(0,1,2);
        v(3,0)=eangc(0);
        v(4,0)=eangc(1);
        v(5,0)=eangc(2);
        v=v/dt;
        return v;
    }

    Eigen::Affine3d get_transform_from_values(Eigen::AngleAxisd anax, double x,double y,double z)
    {
        Eigen::Affine3d t;
        Eigen::Translation3d tr(x,y,z);
        t=tr*anax;
        return t;
    }

    Eigen::Matrix<double,3,1> getTransl(Eigen::Affine3d tcurrent)
    {
        
        Eigen::Matrix<double,3,1> v;
        v(0,0)=tcurrent.translation().x();
        v(1,0)=tcurrent.translation().y();
        v(2,0)=tcurrent.translation().z();
        return v;
    }

    Eigen::Matrix<double,3,1> getRPYs(Eigen::Affine3d tcurrent)
    {
        Eigen::Matrix<double,3,1> v;
        Eigen::Vector3d eangc=tcurrent.rotation().eulerAngles(0,1,2);
        v(0,0)=eangc(0);
        v(1,0)=eangc(1);
        v(2,0)=eangc(2);
        return v;
    }

    void js(const sensor_msgs::JointStatePtr& joint_states)
    {
        joint_pos.resize(chainlength);
        for(int i=0;i<chainlength;i++)
        {
            joint_pos(i)=joint_states->position[i];
        }
    }

    Eigen::Matrix<double,3,1> p(double s,Eigen::Matrix<double,3,1> pi,Eigen::Matrix<double,3,1> pf)
    {
        Eigen::Matrix<double,3,1> den=pf-pi;
        Eigen::Matrix<double,3,1> ps=pi+(s*den)/den.norm();
        return ps;
    }

    geometry_msgs::Twist rectilinear_path_trap(ros::Publisher *pub,Eigen::Affine3d start,Eigen::Affine3d goal)
    {
        
        geometry_msgs::Twist t;
        Eigen::Matrix<double,3,1> pi=getTransl(start);
        Eigen::Matrix<double,3,1> pf=getTransl(goal);
        Eigen::Matrix<double,3,1> qi=getRPYs(start);
        Eigen::Matrix<double,3,1> qf=getRPYs(goal);
        Eigen::Matrix<double,3,1> p0;
        p0 = p(0,pi,pf);
        double L=(pf-pi).norm();
        double dt=L/100;
        double i=0;
        while(ros::ok()&&i<100)
        {  
            p0 = p(i,pi,pf);
            i=i+dt;
            t.linear.x=p0[0]/10;
            t.linear.y=p0[1]/10;
            t.linear.z=p0[2]/10;
            t.angular.x=0;
            t.angular.y=0;
            t.angular.z=0;
            pub->publish(t);
            ros::Duration(dt).sleep();
            ros::spinOnce();
        }
        t.linear.x=0;
        t.linear.y=-0.05;
        t.linear.z=0;
        t.angular.x=0;
        t.angular.y=0;
        t.angular.z=0;
        return t;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_publiher");
    ros::NodeHandle n;
    ros::Rate r(100);   
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/command", 100);
    kdl_test k(&n,"three_dof_planar_link0","three_dof_planar_eef");
    ros::Duration(1).sleep();
    ros::spinOnce();
    Eigen::Affine3d t_start=k.KDL_to_Eigen(k.solveForwardKinematics());
    Eigen::Affine3d t_goal;
    Eigen::Vector3d axis(0,0,1);
    Eigen::AngleAxisd anax(0,axis);
    t_goal=k.get_transform_from_values(anax,1,0,0.05);
    geometry_msgs::Twist t=k.rectilinear_path_trap(&pub,t_start,t_goal);
    while(ros::ok())
    {  
        //std::cout<<k.solveForwardKinematics()<<std::endl;
        pub.publish(t);
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    
    return 0;
}