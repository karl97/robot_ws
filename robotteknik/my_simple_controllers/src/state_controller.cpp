#include "../include/HomogeneousTransformationClass.h"
#include <tf/transform_broadcaster.h>

#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {

void StateController::update(const ros::Time& time, const ros::Duration& period) {
   //ROS_INFO("State Controller: here read joint handles, compute forward kinematic model and publish tool tip pose to TF");
   double list[numOfJoints];
   for (int i=0; i<numOfJoints; i++)
   {
      list[i] = jointStateHolder[i].getPosition();
   }
  HomogeneousTransformation T(Matrix4d::Identity());
  Vector3d aor(0,0,1);
  double a,b,c,d,e,f;
  a=0.5+0.05;
  b=0.3;
  c=0.2;
  d=list[0];
  e=list[1];
  f=list[2];
  Vector3d va(a,0,0);
  Vector3d vb(b,0,0);
  Vector3d vc(c,0,0);
  Link base_link;
  base_link.setPoseOffsetFromParent(Matrix4d::Identity());
  Link L0;
  HomogeneousTransformation TL0(T.getRotTransformZ(M_PI/4.0));
  TL0.setTranslation(va);
  L0.setPoseOffsetFromParent(TL0.getTransformation());
  Link L1;
  HomogeneousTransformation TL1(T.getRotTransformZ(-M_PI/8.0));
  TL1.setTranslation(vb);
  L1.setPoseOffsetFromParent(TL1.getTransformation());
  Link L2;
  HomogeneousTransformation TL2(T.getRotTransformZ(-M_PI/8.0));
  TL2.setTranslation(vc);
  L2.setPoseOffsetFromParent(TL2.getTransformation());
  Joint q0;
  q0.setAxisOfRot(aor);
  q0.setJointValue(d);
  Joint q1;
  q1.setAxisOfRot(aor);
  q1.setJointValue(e);
  Joint q2;
  q2.setAxisOfRot(aor);
  q2.setJointValue(f);

  base_link.setChild(&q0);
  q0.setParent(&base_link);
  q0.setChild(&L0);

  L0.setParent(&q0);
  L0.setChild(&q1);

  q1.setParent(&L0);
  q1.setChild(&L1);

  L1.setParent(&q1);
  L1.setChild(&q2);

  q2.setParent(&L1);
  q2.setChild(&L2);

  L2.setParent(&q2);
  
  Chain chain;
  chain.setBase(&base_link);
  
  tf::TransformBroadcaster broadcaster;
  HomogeneousTransformation TB0 = chain.getLinkPose(1);
  HomogeneousTransformation TB1 = chain.getLinkPose(2);
  HomogeneousTransformation TB2 = chain.getLinkPose(3);
   HomogeneousTransformation T0B(TB0.inverse().getTransformation());
   HomogeneousTransformation T1B(TB1.inverse().getTransformation());
   HomogeneousTransformation T01(T0B.applyTransformationOnTransformation(TB1).getTransformation());
   HomogeneousTransformation T12(T1B.applyTransformationOnTransformation(TB2).getTransformation());
  Vector4d qu0=TB0.getQuaternion();
  Vector4d qu1=T01.getQuaternion();
  Vector4d qu2=T12.getQuaternion();

  Vector3d p0=TB0.getTranslation();
  Vector3d p1=T01.getTranslation();
  Vector3d p2=T12.getTranslation();
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu0(1), qu0(2), qu0(3), qu0(0)), tf::Vector3(p0(0), p0(1), p0(2))), ros::Time::now(),"base_link", "L0"));
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu1(1), qu1(2), qu1(3), qu1(0)), tf::Vector3(p1(0), p1(1), p1(2))), ros::Time::now(),"L0", "L1"));
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu2(1), qu2(2), qu2(3), qu2(0)), tf::Vector3(p2(0), p2(1), p2(2))), ros::Time::now(),"L1", "L2"));
}

bool StateController::init(hardware_interface::JointStateInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
   ROS_INFO("State Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");
   
   const std::vector<std::string>& jointNames = hw->getNames();
   numOfJoints = jointNames.size();
   for (int i=0; i<numOfJoints; i++)
   {
      ROS_INFO("Got joint %s", jointNames[i].c_str());
   }
   for (int i=0; i<numOfJoints; i++)
   {
      jointStateHolder.push_back(hw->getHandle(jointNames[i]));
   }

   return true;


}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
                       controller_interface::ControllerBase)