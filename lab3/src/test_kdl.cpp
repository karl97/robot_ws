#include <kdl_parser/kdl_parser.hpp>
#include "ros/ros.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <ctime>
#include <cstdlib>
#include <Eigen/Geometry> 


KDL::JntArray joint_pos;
int chainlength=0;

class kdl_test
{
    KDL::Tree my_tree;
    KDL::Chain chain;
    ros::NodeHandle *nh;
public:
    kdl_test(ros::NodeHandle *n,std::string base_link,std::string tip_link)//sets up the chain
    {
        nh=n;
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
        if(chainlength==0)
        {
            chainlength=chain.getNrOfJoints();
        }
    };
    
    KDL::Frame solveForwardKinematics()
    {
        KDL::Frame cart_pos;
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        fk_solver.JntToCart(joint_pos, cart_pos);
        return cart_pos;
    };

    Eigen::Matrix< double, 6, 3 > jacobian()
    {
        KDL::Jacobian  J;
        KDL::ChainJntToJacSolver jac_solver(chain);
        J.resize(chain.getNrOfJoints());
        jac_solver.JntToJac(joint_pos,J);
        return J.data;;
    };

    Eigen::Matrix<double,3,6> DLSInv(Eigen::Matrix< double, 6, Eigen::Dynamic > J , double k)
    {
        Eigen::Matrix<double,3,6> Jt=J.transpose();
        Eigen::Matrix<double,6,6> JJtInv=(J*Jt+k*k*Eigen::Matrix<double,6,6>::Identity()).inverse();
        return Jt*JJtInv;
    }

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

    Eigen::Affine3d get_transform_from_values(Eigen::AngleAxisd anax, double x,double y,double z)
    {
        Eigen::Affine3d t;
        Eigen::Translation3d tr(x,y,z);
        t=tr*anax;
        return t;
    }

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

    const sensor_msgs::JointState IKSolver(ros::Publisher *pub, std::string goal_link, Eigen::Affine3d goal_pose)//std::string link_frame_name, KDL::JntArray goal_pos)
    {
        
        Eigen::Vector3d p0;
        srand(time(0));
        p0(0)=guess(-1.9, 1.9);
        p0(1)=guess(-1.9, 1.9);
        p0(2)=guess(-1.9, 1.9);
        std::cout<<"starting guess: "<<p0(0)<<":"<<p0(1)<<":"<<p0(2)<<std::endl;
        kdl_test kshort(nh,"three_dof_planar_link0",goal_link);
        KDL::Frame eef_pos = kshort.solveForwardKinematics();
        Eigen::Affine3d T = KDL_to_Eigen(eef_pos);
        Eigen::Affine3d shortest_T=T.inverse()*goal_pose;
        double translation_threshold=0.001;
        double dt=0.001;
        int counter=0;
        Eigen::Matrix<double,6,1> vel=getVelocity(shortest_T, 1);

        sensor_msgs::JointState jointstatemsg;
        jointstatemsg.name.clear();
        jointstatemsg.name.push_back("three_dof_planar_joint1");
        jointstatemsg.name.push_back("three_dof_planar_joint2");
        jointstatemsg.name.push_back("three_dof_planar_joint3");
        jointstatemsg.position.resize(jointstatemsg.name.size());

        while(ros::ok()&&(counter<10000)&&(vel.norm()>translation_threshold))
        {
            Eigen::Matrix<double,3,6> Jps = kshort.DLSInv(kshort.jacobian(),0.001);
            
            p0=p0+dt*Jps*vel;
            //std::cout<<counter<<std::endl;
            eef_pos = kshort.solveForwardKinematics();
            T = KDL_to_Eigen(eef_pos);
            shortest_T=T.inverse()*goal_pose;
            vel=getVelocity(shortest_T, 1);
            counter++;

            jointstatemsg.position[0]=p0(0);
            jointstatemsg.position[1]=p0(1);
            jointstatemsg.position[2]=p0(2);
            pub->publish(jointstatemsg);
            ros::Duration(0.001).sleep();
            ros::spinOnce();
        }
        std::cout<<counter<<std::endl;
        return jointstatemsg;
    };

    double guess(double minrange,double maxrange)
{
    double length;
    if(minrange<0&&maxrange>0)
    {
        length=-minrange+maxrange;
    }
    else if(minrange>0)
    {
        length=maxrange-minrange;
    }
    else
    {
        length=-minrange+maxrange;
    }
    
    double randomValueInRange = double(rand())/double((RAND_MAX)) * length+minrange;;
 
    return randomValueInRange;
}

};

void js(const sensor_msgs::JointStatePtr& joint_states)
{
    joint_pos.resize(chainlength);
   for(int i=0;i<chainlength;i++)
    {
        joint_pos(i)=joint_states->position[i];
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_kdl");
    ros::NodeHandle n;
    ros::Rate r(100);
    kdl_test k(&n,"three_dof_planar_link0","three_dof_planar_eef");    
    ros::Subscriber sub = n.subscribe("joint_states", 1000, js);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);

    Eigen::Affine3d t_goal;
    Eigen::Vector3d axis(0,0,1);
    Eigen::AngleAxisd anax(M_PI/2.0,axis);
    t_goal=k.get_transform_from_values(anax,0.5,0.5,0.05);
    sensor_msgs::JointState jointstatemsg = k.IKSolver(&pub,"three_dof_planar_eef",t_goal);
    
    while(ros::ok())
    {
       // sensor_msgs::JointState jointstatemsg = k.IKSolver(&pub,"three_dof_planar_eef");
        //std::cout<<k.jacobian()<<std::endl;
        ROS_INFO("publishing final robot joint config");
        pub.publish(jointstatemsg);
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    
    return 0;
}