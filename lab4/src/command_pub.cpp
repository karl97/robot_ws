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
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>

std::ofstream myfile;
const char *filename=NULL;
std::ofstream myfile2;
const char *filename2=NULL;


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

    double q(double t,double tf, double qi ,double qf,double qspeed)
    {   
        double tc=(qi-qf+qspeed*tf)/qspeed;
        double qacc=(qspeed*qspeed)/(qi-qf+qspeed*tf);
        double val=std::abs(qf-qi)/tf;
        if(val>=std::abs(qspeed))
        {
            std::cout<<"to small speed must be over..."<<val<<std::endl;
            return 0;
        }
        if(std::abs(qspeed)>2*val)
        {
            std::cout<<"to large speed must be smaller than..."<<2*val<<std::endl;
            return 0;
        }
        double qt=0;
        if((0<=t)&&(t<=tc))
        {
            qt=qi+0.5*qacc*t*t;
            //std::cout<<"1"<<std::endl;
        }
        if((tc<t)&&(t<=(tf-tc)))
        {
            qt=qi+qacc*tc*(t-0.5*tc);
            //std::cout<<"2"<<std::endl;
        }
        if(((tf-tc)<t)&&(t<=tf))
        {
            qt=qf-0.5*qacc*(tf-t)*(tf-t);
            //std::cout<<"3"<<std::endl;
        }
        //std::cout<<qt<<std::endl;
        return qt;
    }

    double q2(double t,double tf, double qi ,double qf,double qacc)
    {   
        double tc=tf*0.5-0.5*sqrt((tf*tf*qacc-4*(qf-qi))/qacc);
        double val=4*std::abs(qf-qi)/(tf*tf);
        if(val>std::abs(qacc))
        {
            std::cout<<"to small acc..."<<std::endl;
        }
       
        double qt=0;
        if((0<=t)&&(t<=tc))
        {
            qt=qi+0.5*qacc*t*t;
            std::cout<<"1"<<std::endl;
        }
        if((tc<t)&&(t<=(tf-tc)))
        {
            qt=qi+qacc*tc*(t-0.5*tc);
            std::cout<<"2"<<std::endl;
        }
        if(((tf-tc)<t)&&(t<=tf))
        {
            qt=qf-0.5*qacc*(tf-t)*(tf-t);
            std::cout<<"3"<<std::endl;
        }
        //std::cout<<qt<<std::endl;
        return qt;
    }

    geometry_msgs::Twist rectilinear_path_trap(ros::Publisher *pub,Eigen::Affine3d start,Eigen::Affine3d goal,double pspeed,double qspeed)
    {
        geometry_msgs::Twist t;
        Eigen::Matrix<double,3,1> pi=getTransl(start);
        Eigen::Matrix<double,3,1> pf=getTransl(goal);
        Eigen::Matrix<double,3,1> qi=getRPYs(start);
        Eigen::Matrix<double,3,1> qf=getRPYs(goal);
        double L=(pf-pi).norm();
        double L2=(qf-qi).norm();
        double dt=0.01;
        double curr_time=0;
        double tf=1.0;
        double trapT=0;
        int numSamples=int(tf/dt)+1;
        double trapSamples[numSamples];
        double trapSamples2[numSamples];
        for(int i=0;i<numSamples;i+=1)
        {
            if(i==numSamples-1)
            {
                curr_time=tf;
            }
            trapSamples[i]=q(curr_time,tf,0,L,pspeed);
            trapSamples2[i]=q(curr_time,tf,0,L2,qspeed);
            
            //std::cout<<trapSamples[i]<<":"<<curr_time<<std::endl;
            curr_time+=dt;
        }
        Eigen::Matrix<double,3,1> pvel;
        Eigen::Matrix<double,3,1> pc;
        Eigen::Matrix<double,3,1> p_prev=pi;

        Eigen::Matrix<double,3,1> qvel;
        Eigen::Matrix<double,3,1> qc;
        Eigen::Matrix<double,3,1> q_prev=qi;

        double start_time=ros::Time::now().toSec();
        int i=0;
        curr_time=0;
        while((i<numSamples)&&ros::ok())
        {  
            if((ros::Time::now().toSec() - start_time >=curr_time)&&(i<numSamples))
            {
                pc=p(trapSamples[i],pi,pf);
                pvel=(pc-p_prev)/0.1;
                p_prev=pc;

                qc=p(trapSamples2[i],qi,qf);
                qvel=(qc-q_prev)/0.1;
                q_prev=qc;
                Eigen::Affine3d realT = KDL_to_Eigen(solveForwardKinematics());
                Eigen::Matrix<double,3,1> realTransl = realT.translation();
                Eigen::Matrix<double,3,1> realRot = getRPYs(realT);

                Eigen::Matrix<double,3,1> diffTransl = realTransl-pc;
                Eigen::Matrix<double,3,1> diffRot = realRot - qc;

                std::ostringstream ss;
                ss << diffTransl.norm();
                std::string s(ss.str());
                myfile << s <<",";

                std::ostringstream ss2;
                ss2 << diffRot.norm();
                std::string s2(ss2.str());
                myfile2 << s2 <<",";

                t.linear.x=pvel[0];
                t.linear.y=pvel[1];
                t.linear.z=pvel[2];
                t.angular.x=qvel[0];
                t.angular.y=qvel[1];
                t.angular.z=qvel[2];
                pub->publish(t);
                i+=1;
                curr_time+=0.1;
            }
            ros::spinOnce();
        }
        t.linear.x=0;
        t.linear.y=0;
        t.linear.z=0;
        t.angular.x=0;
        t.angular.y=0;
        t.angular.z=0;
        pub->publish(t);
        return t;
    }

};

int main(int argc, char** argv)
{

    filename=argv[1];
    myfile.open (filename);
    filename2=argv[2];
    myfile2.open (filename2);
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
    Eigen::AngleAxisd anax(-M_PI/2.0,axis);
    Eigen::AngleAxisd anax2(M_PI/2.0,axis);
    Eigen::AngleAxisd anax3(0.0,axis);
    t_goal=k.get_transform_from_values(anax,0.3,0.3,0.05);
    k.rectilinear_path_trap(&pub,t_start,t_goal,0.9,1.6);
    std::cout<<k.solveForwardKinematics()<<std::endl;
    ros::Duration(1).sleep();

    t_start=k.KDL_to_Eigen(k.solveForwardKinematics());
    t_goal=k.get_transform_from_values(anax2,0.0,1.0,0.05);
    k.rectilinear_path_trap(&pub,t_start,t_goal,0.9,3.2);
    std::cout<<k.solveForwardKinematics()<<std::endl;
    ros::Duration(1).sleep();

    t_start=k.KDL_to_Eigen(k.solveForwardKinematics());
    t_goal=k.get_transform_from_values(anax3,1.0,0.0,0.05);
    k.rectilinear_path_trap(&pub,t_start,t_goal,1.5,3.0); 
    std::cout<<k.solveForwardKinematics()<<std::endl;
    
    /*while(ros::ok())
    {  
        ros::Duration(1).sleep();
        ros::spinOnce();
    }*/
    myfile.close();
    myfile2.close();
    return 0;
}