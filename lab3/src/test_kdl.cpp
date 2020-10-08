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


KDL::JntArray joint_pos;
int chainlength;

class kdl_test
{
    KDL::Tree my_tree;
    KDL::Chain chain;
public:
    kdl_test(ros::NodeHandle *n)//sets up the chain
    {
        std::string robot_desc_string;
        n->param("robot_description", robot_desc_string, std::string());
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

const sensor_msgs::JointState IKSolver(ros::Publisher *pub)//std::string link_frame_name, KDL::JntArray goal_pos)
{
    KDL::JntArray p0;
    srand(time(0));
    p0.resize(chainlength);
    p0(0)=guess(-1.9, 1.9);
    p0(1)=guess(-1.9, 1.9);
    p0(2)=guess(-1.9, 1.9);
    std::cout<<"starting guess: "<<p0(0)<<":"<<p0(1)<<":"<<p0(2)<<std::endl;
    sensor_msgs::JointState jointstatemsg;
    jointstatemsg.name.clear();
    jointstatemsg.name.push_back("three_dof_planar_joint1");
    jointstatemsg.name.push_back("three_dof_planar_joint2");
    jointstatemsg.name.push_back("three_dof_planar_joint3");
    jointstatemsg.position.resize(jointstatemsg.name.size());
    
    
    
    jointstatemsg.position[0]=p0(0);
    jointstatemsg.position[1]=p0(1);
    jointstatemsg.position[2]=p0(2);
    pub->publish(jointstatemsg);
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
    kdl_test k(&n);    
    ros::Subscriber sub = n.subscribe("joint_states", 1000, js);
    //ros::ServiceServer service = n.advertiseService("IKSolver", IKSolver);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
    while(ros::ok())
    {
        //std::cout<<k.solveForwardKinematics()<<std::endl;   
        //std::cout<<k.jacobian()<<std::endl;
        k.IKSolver(&pub);
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    
    return 0;
}