#ifndef HOMOGENEOUSTRANSFORMATIONCLASS_H
#define HOMOGENEOUSTRANSFORMATIONCLASS_H
#include <Eigen/Eigen>
#include <stdio.h>
#include <iostream>
using namespace Eigen;

class eulerZYX
{
    float z;
    float y;
    float x;
public:
    eulerZYX(float zIn, float yIn, float xIn){z=zIn;y=yIn;x=xIn;}
    float getZ(){return z;}
    float getY(){return y;}
    float getX(){return x;}
};

class rollPitchYaw
{
    float roll;
    float pitch;
    float yaw;
public:
    rollPitchYaw(float r, float p, float y){roll=r;pitch=p;yaw=y;}
    float getRoll(){return roll;}
    float getPitch(){return pitch;}
    float getYaw(){return yaw;}
};

class AngleAxisClass
{
    float angle;
    Vector3d axis;
public:
    float getAngle(){return angle;}
    Vector3d getAxis(){return axis;}
    AngleAxisClass(float b,Vector3d a){axis=a; angle=b;}
    void setAngle(float an){angle=an;}
};

class HomogeneousTransformation
{
    Matrix4d transformation;
public: 
    Matrix4d getRotTransformX(float angle);
    Matrix4d getRotTransformY(float angle);
    Matrix4d getRotTransformZ(float angle);
    Vector3d getTranslation();
    void setTranslation(Vector3d v);
    void setTransformation(Matrix4d t){transformation=t;}
    Matrix4d getTransformation(){return transformation;}
    HomogeneousTransformation(){};
    HomogeneousTransformation(Matrix4d T);
    HomogeneousTransformation inverse();
    Vector4d applyTransformationOnVector(Vector4d point);
    HomogeneousTransformation applyTransformationOnTransformation(HomogeneousTransformation transformationToApplyTo);
    HomogeneousTransformation identity();
    HomogeneousTransformation eulerZYXTransformation(eulerZYX e);
    rollPitchYaw getRPY();
    void printTransformationToTerminal();
    AngleAxisClass getAngleAxis();
    HomogeneousTransformation angleAxisTransformation(AngleAxisClass anAx);
    Vector4d getQuaternion();
    HomogeneousTransformation quaternionTransformation(Vector4d q);
};

class Link;

class Joint
{
    Vector3d axisOfRot;
    double jointValue;
    Link *parent=NULL;
    Link *child=NULL;
public:
    void setAxisOfRot(Vector3d v){axisOfRot = v;};
    void setJointValue(double q){jointValue = q;};
    void setParent(Link *p){parent = p;};
    void setChild(Link *c){child = c;};
    double getJointValue(){return jointValue;};
    Link* getParent(){return parent;};
    Link* getChild(){return child;};
};

class Link
{
    Joint *parent=NULL;
    Joint *child=NULL;
    HomogeneousTransformation poseOffsetFromParent;
public:
    void setParent(Joint *p){parent = p;};
    void setChild(Joint *c){child = c;};
    Joint* getParent(){return parent;};
    Joint* getChild(){return child;};
    void setPoseOffsetFromParent(Matrix4d T){poseOffsetFromParent.setTransformation(T);};
    HomogeneousTransformation getPoseOffsetFromParent(){return poseOffsetFromParent;}
};

class Chain
{
    Link *base=NULL;
public:
    void setBase(Link *b){base = b;};
    void printChainToTerminal();
    void printLinkPosesToTerminal();
};

#endif