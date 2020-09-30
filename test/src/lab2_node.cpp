#include "ros/ros.h"
#include "../include/HomogeneousTransformationClass.h"
#include <sstream>


void test_chain()
{
  HomogeneousTransformation T(Matrix4d::Identity());
  Vector3d aor(0,0,1);
  double a,b,c,d,e,f;
  a=1;
  b=1;
  c=1;
  d=0;
  e=0;
  f=0;
  Vector3d va(a,0,0);
  Vector3d vb(b,0,0);
  Vector3d vc(c,0,0);
  Link base_link;
  base_link.setPoseOffsetFromParent(Matrix4d::Identity());
  Link L0;
  T.setTranslation(va);
  L0.setPoseOffsetFromParent(T.getTransformation());
  Link L1;
  T.setTranslation(vb);
  L1.setPoseOffsetFromParent(T.getTransformation());
  Link L2;
  T.setTranslation(vc);
  L2.setPoseOffsetFromParent(T.getTransformation());
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  test_chain();

  int count = 0;
  
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}