#include <tf/transform_broadcaster.h>
#include <my_simple_controllers/velocity_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin


namespace my_simple_controllers {

void VelocityController::control(const geometry_msgs::Twist control)
{
    desired_cart_vel(0,0)=control.linear.x;
    desired_cart_vel(1,0)=control.linear.y;
    desired_cart_vel(2,0)=control.linear.z;
    desired_cart_vel(3,0)=control.angular.x;
    desired_cart_vel(4,0)=control.angular.y;
    desired_cart_vel(5,0)=control.angular.z;
}

void VelocityController::update(const ros::Time& time, const ros::Duration& period) 
{
    KDL::JntArray joint_pos;
    joint_pos.resize(chain.getNrOfJoints());
    for (int i=0; i<numOfJoints; i++)
    {  
        joint_pos(i) = velocityHolder[i].getPosition();
    }
    KDL::Jacobian  J;
    KDL::ChainJntToJacSolver jac_solver(chain);
    J.resize(chain.getNrOfJoints());
    jac_solver.JntToJac(joint_pos,J);
    double dt = double(period.sec)+double(period.nsec)*1e-9;
    Eigen::Matrix< double, 6, 3 > jac=J.data;
    Eigen::Matrix<double,6,1> cart_pos=prev_cart_pos+desired_cart_vel*dt;
    prev_cart_pos=cart_pos;
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame fk_pos;
    fk_solver.JntToCart(joint_pos, fk_pos);
    Eigen::Matrix<double,6,1> fk=frameToEigen(fk_pos);
    Eigen::Matrix<double,6,1> e=cart_pos-fk;
    Eigen::Matrix<double,6,1> u=Kp[0]*e+desired_cart_vel;
    Eigen::Matrix<double,3,6> Jinv=DLSInv(jac,0.001);
    Eigen::Matrix<double,3,1> qdot=Jinv*u;
    velocityHolder[0].setCommand(qdot(0,0));
    velocityHolder[1].setCommand(qdot(1,0));
    velocityHolder[2].setCommand(qdot(2,0));
    //std::cout<<linearvels[0]<<std::endl;
    //ROS_INFO("Got joint vel %f", vel[0]);
}
   

bool VelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) 
{
    ROS_INFO("Velocity Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");
    nh=&root_nh;
    sub = nh->subscribe("/command", 1000, &VelocityController::control,this);
    const std::vector<std::string>& jointNames = hw->getNames();
    numOfJoints = jointNames.size();
    for (int i=0; i<numOfJoints; i++)
    {
        ROS_INFO("Got joint %s", jointNames[i].c_str());
    }
    for (int i=0; i<numOfJoints; i++)
    {
        velocityHolder.push_back(hw->getHandle(jointNames[i]));
        double gain;
        //ROS_INFO("Got joint %s", jointNames[i].c_str());
        controller_nh.param<double>("/my_velocity_controller/gains/"+jointNames[i]+"/p",gain,1.0);
        Kp.push_back(gain);
        ROS_INFO("Got gains %lf", Kp[i]);
    }

    std::string base_link = "three_dof_planar_link0";
    std::string tip_link = "three_dof_planar_eef";
    std::string robot_desc_string;
    root_nh.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_INFO("Failed to construct kdl tree");     
    }
    else
    {
        ROS_INFO("succeded to construct kdl tree"); 
    }
    if (!my_tree.getChain(base_link, tip_link, chain))
    {
        ROS_INFO("Couldn't find chain from %s to %s\n", base_link.c_str(), tip_link.c_str());
    }
    else
    {
        ROS_INFO("succeded to find chain from %s to %s\n", base_link.c_str(), tip_link.c_str());
    }
    KDL::JntArray joint_pos;
    joint_pos.resize(chain.getNrOfJoints());
    for (int i=0; i<numOfJoints; i++)
    {  
        joint_pos(i) = velocityHolder[i].getPosition();
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame cart_pos;
    fk_solver.JntToCart(joint_pos, cart_pos);
    prev_cart_pos=frameToEigen(cart_pos);
    return true;
}   

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::VelocityController,
                       controller_interface::ControllerBase)