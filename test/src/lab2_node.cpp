#include "ros/ros.h"
#include "../include/HomogeneousTransformationClass.h"
#include <sstream>
#include <tf/transform_broadcaster.h>


void test_chain()
{
  HomogeneousTransformation T(Matrix4d::Identity());
  Vector3d aor(0,0,1);
  double a,b,c,d,e,f;
  a=0.5+0.05;
  b=0.3;
  c=0.2;
  d=M_PI/2;
  e=-M_PI/8;
  f=-M_PI/8;
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
  chain.printLinkPosesToTerminal();
}

void publish_tfs()
{
  
  HomogeneousTransformation T(Matrix4d::Identity());
  Vector3d aor(0,0,1);
  double a,b,c,d,e,f;
  a=0.5+0.05;
  b=0.3;
  c=0.2;
  d=M_PI/2;
  e=-M_PI/8;
  f=-M_PI/8;
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
  HomogeneousTransformation T0 = chain.getLinkPose(1);
  HomogeneousTransformation T1 = chain.getLinkPose(2);
  HomogeneousTransformation T2 = chain.getLinkPose(3);

  Vector4d qu0=T0.getQuaternion();
  Vector4d qu1=T1.getQuaternion();
  Vector4d qu2=T2.getQuaternion();

  Vector3d p0=T0.getTranslation();
  Vector3d p1=T1.getTranslation();
  Vector3d p2=T2.getTranslation();
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu0(1), qu0(2), qu0(3), qu0(0)), tf::Vector3(p0(0), p0(1), p0(2))), ros::Time::now(),"base_link", "L0"));
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu1(1), qu1(2), qu1(3), qu1(0)), tf::Vector3(p1(0), p1(1), p1(2))), ros::Time::now(),"base_link", "L1"));
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(qu2(1), qu2(2), qu2(3), qu2(0)), tf::Vector3(p2(0), p2(1), p2(2))), ros::Time::now(),"base_link", "L2"));
}  

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;
  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  test_chain();

  while(n.ok()){
    publish_tfs();
    r.sleep();
  
  }
}