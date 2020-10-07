#include <kdl_parser/kdl_parser.hpp>
#include "ros/ros.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>

KDL::JntArray joint_pos;
int chainlength;

class kdl_test
{
    KDL::Tree my_tree;
    KDL::Chain chain;
public:
    kdl_test(ros::NodeHandle n)//sets up the chain
    {
        std::string robot_desc_string;
        n.param("robot_description", robot_desc_string, std::string());
        if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
        {
            ROS_INFO("Failed to construct kdl tree");     
        }
        ROS_INFO("succeded to construct kdl tree"); 
        std::string base_link = "three_dof_planar_link0";
        std::string tip_link  = "three_dof_planar_eef";

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
    Eigen::Matrix< double, 6, Eigen::Dynamic > jacobian()
    {
        KDL::Jacobian  J;
        KDL::ChainJntToJacSolver jac_solver(chain);
        J.resize(chain.getNrOfJoints());
        jac_solver.JntToJac(joint_pos,J);
        return J.data;;
    };
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
    kdl_test k(n);    
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, js);
    while(ros::ok)
    {
        //std::cout<<k.solveForwardKinematics()<<std::endl;   
        std::cout<<k.jacobian()<<std::endl;
        sleep(1);
        ros::spinOnce();
    }
    
    return 0;
}