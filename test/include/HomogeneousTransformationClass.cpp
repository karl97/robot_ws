#include "HomogeneousTransformationClass.h"

Matrix4d HomogeneousTransformation::getRotTransformX(float angle)
{
    Matrix4d rotationX= Matrix4d::Identity(4,4);
    rotationX(1,1)=cos(angle);
    rotationX(1,2)=-sin(angle);
    rotationX(2,1)=sin(angle);
    rotationX(2,2)=cos(angle);
    return rotationX;
}

Matrix4d HomogeneousTransformation::getRotTransformY(float angle)
{
    Matrix4d rotationY= Matrix4d::Identity(4,4);
    rotationY(0,0)=cos(angle);
    rotationY(2,0)=-sin(angle);
    rotationY(2,2)=cos(angle);
    rotationY(0,2)=sin(angle);
    return rotationY;
}

Matrix4d HomogeneousTransformation::getRotTransformZ(float angle)
{
    Matrix4d rotationZ= Matrix4d::Identity(4,4);
    rotationZ(0,0)=cos(angle);
    rotationZ(0,1)=-sin(angle);
    rotationZ(1,0)=sin(angle);
    rotationZ(1,1)=cos(angle);
    return rotationZ;
}

void HomogeneousTransformation::setTranslation(Vector3d v)
{
    transformation(0,3)=v(0);
    transformation(1,3)=v(1);
    transformation(2,3)=v(2);
}

Vector3d HomogeneousTransformation::getTranslation()
{
    Vector3d v;
    v(0)=transformation(0,3);
    v(1)=transformation(1,3);
    v(2)=transformation(2,3);
    return v;
}

HomogeneousTransformation::HomogeneousTransformation(Matrix4d T)
{
    transformation = T;
}

HomogeneousTransformation HomogeneousTransformation::inverse()
{
    Matrix4d t = Matrix4d::Identity(4,4);
    for(int i=0;i<3;i++)
    {
        t(i,3) = -transformation(i,3);
    }

    Matrix4d R = Matrix4d::Identity(4,4);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R(i,j) = transformation(i,j);
        }
    }

    Matrix4d Rt = R.transpose();
    Matrix4d inverseTransformation = Rt * t;
    HomogeneousTransformation finalTransformation(inverseTransformation);
    
    return finalTransformation;
}

Vector4d HomogeneousTransformation::applyTransformationOnVector(Vector4d point)
{
    Vector4d transformedPoint = transformation * point;
    return transformedPoint;
}

HomogeneousTransformation HomogeneousTransformation::applyTransformationOnTransformation(HomogeneousTransformation transformationToApplyTo)
{
    HomogeneousTransformation newTransform(transformation * transformationToApplyTo.transformation );
    return newTransform;
}

HomogeneousTransformation HomogeneousTransformation::identity()
{
    HomogeneousTransformation identityTransform(Matrix4d::Identity(4,4));
    return identityTransform;
}

HomogeneousTransformation HomogeneousTransformation::eulerZYXTransformation(eulerZYX eulerAngles)
{
    Matrix4d Z = getRotTransformZ(eulerAngles.getZ());
    Matrix4d Y = getRotTransformY(eulerAngles.getY());
    Matrix4d X = getRotTransformX(eulerAngles.getX());
    Matrix4d eulerTransform=Z*Y*X;
    HomogeneousTransformation euler(eulerTransform);
    return euler;
}

rollPitchYaw HomogeneousTransformation::getRPY()
{
    float temp = sqrt(pow(transformation(0,0),2)+pow(transformation(1,0),2));
    bool singular = temp < 1e-6;
    if(!singular){
        float alpha = atan2(transformation(1,0),transformation(0,0));
        float beta = atan2(-transformation(2,0),temp);
        float gamma = atan2(transformation(2,1),transformation(2,2));
        rollPitchYaw RPYAngles(gamma,beta,alpha);
        return RPYAngles;
    }
    else
    {
        float alpha = 0.0;
        float beta = atan2(-transformation(2,0),temp);
        float gamma = atan2(transformation(1,2),transformation(1,1));
        rollPitchYaw RPYAngles(gamma,beta,alpha);
        return RPYAngles;
    }
}

void HomogeneousTransformation::printTransformationToTerminal()
{
    for(int i=0;i<4;i++)
    {    
        float a=transformation(i,0);
        float b=transformation(i,1);
        float c=transformation(i,2);
        float d=transformation(i,3);
        std::cout<<"["<<a<<","<<b<<","<<c<<","<<d<<"]"<<std::endl;    
    }
}

Vector4d HomogeneousTransformation::getQuaternion()
{
    rollPitchYaw RPY = getRPY();
    double cy = cos(RPY.getYaw() * 0.5);
    double sy = sin(RPY.getYaw() * 0.5);
    double cp = cos(RPY.getPitch() * 0.5);
    double sp = sin(RPY.getPitch() * 0.5);
    double cr = cos(RPY.getRoll() * 0.5);
    double sr = sin(RPY.getRoll() * 0.5);
    Vector4d q;
    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;
    return q;
}

HomogeneousTransformation HomogeneousTransformation::quaternionTransformation(Vector4d q)
{
    Matrix4d m = Matrix4d::Identity(4,4);
    m(0,0) = 1 - 2*q(2)*q(2) - 2*q(3)*q(3);
    m(0,1) = 2*q(1)*q(2) - 2*q(3)*q(0);
    m(0,2) = 2*q(1)*q(3) + 2*q(2)*q(0);
    m(1,0) = 2*q(1)*q(2) + 2*q(3)*q(0);
    m(1,1) = 1 - 2*q(1)*q(1) - 2*q(3)*q(3);
    m(1,2) = 2*q(2)*q(3) - 2*q(1)*q(0);
    m(2,0) = 2*q(1)*q(3) - 2*q(2)*q(0);
    m(2,1) = 2*q(2)*q(3) + 2*q(1)*q(0);
    m(2,2) = 1 - 2*q(1)*q(1) - 2*q(2)*q(2);
    HomogeneousTransformation T(m);
    return T;
}

AngleAxisClass HomogeneousTransformation::getAngleAxis()
{
    Vector4d q = getQuaternion();
    q.normalize();
    float angle = 2.0 * acos(q(0));
    float x,y,z;
    float s = sqrt(1-q(0)*q(0));
    if(s < 0.001)
    {
        x=q(1);
        y=q(2);
        z=q(3);
    }
    else
    {
        x=q(1) / s;
        y=q(2) / s;
        z=q(3) / s;
    }
    Vector3d v(x,y,z);
    AngleAxisClass a(angle,v);
    return a;
}

HomogeneousTransformation HomogeneousTransformation::angleAxisTransformation(AngleAxisClass ax)
{
    Matrix4d m = Matrix4d::Identity(4,4);
    float c = cos(ax.getAngle());
    float s = sin(ax.getAngle());
    float t = 1-c;
    Vector3d normXYZ = ax.getAxis();
    normXYZ.normalize();
    float x = normXYZ(0);
    float y = normXYZ(1);
    float z = normXYZ(2);
    m(0,0) = t*x*x + c;
    m(0,1) = t*x*y - z*s;
    m(0,2) = t*x*z + y*s;
    m(1,0) = t*x*y + z*s;
    m(1,1) = t*y*y + c;
    m(1,2) = t*y*z - x*s;
    m(2,0) = t*x*s - y*s;
    m(2,1) = t*y*z + x*s;
    m(2,2) = t*z*z + c;
    HomogeneousTransformation T(m);
    return T;
}

void Chain::printChainToTerminal()
{
    Joint* j = base->getChild();
    Link *l = j->getChild();
    std::cout<<"base_link"<<std::endl;
    int linkCounter=1;
    int jointCounter=1;
    int linkOrJoint=2;
    while((j!=NULL)&&(l!=NULL))
    {
        if(linkOrJoint % 2 == 0)
        {
            std::cout<<"Joint_"<<jointCounter<<std::endl;
            jointCounter++;
            l = j->getChild();
        }
        else
        {
            std::cout<<"Link_"<<linkCounter<<std::endl;
            linkCounter++;
            j = l->getChild();
        }
        linkOrJoint++;

    }
}

void Chain::printLinkPosesToTerminal()
{
    Joint* j = base->getChild();
    Link *l = j->getChild();
    std::cout<<"base_link"<<std::endl;
    HomogeneousTransformation T = base->getPoseOffsetFromParent();
    HomogeneousTransformation Txx=T;
    Txx.printTransformationToTerminal();
    int linkCounter=1;
    int jointCounter=1;
    int linkOrJoint=2;
    double rotationOnJoint;
    
    while((j!=NULL)&&(l!=NULL))
    {
        if(linkOrJoint % 2 == 0)
        {
            std::cout<<"Joint_"<<jointCounter<<std::endl;
            jointCounter++;
            l = j->getChild();
            rotationOnJoint=j->getJointValue();
        }
        else
        {
            std::cout<<"Link_"<<linkCounter<<std::endl;
            linkCounter++;
            j = l->getChild();
            T.setTransformation(T.getRotTransformZ(rotationOnJoint));
            Txx=Txx.applyTransformationOnTransformation(T).applyTransformationOnTransformation(l->getPoseOffsetFromParent());
            Txx.printTransformationToTerminal();
        }
        linkOrJoint++;

    }
}
