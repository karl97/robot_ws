#include "ros/ros.h"
#include <geometry_msgs/Wrench.h>
#include<stdio.h>
#include <termios.h>  
#include <unistd.h> 

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);         
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                    
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); 

  int c = getchar();  

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt); 
  return c;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "keyboard_node");
    ros::NodeHandle n;
    ros::Rate r(100);   
    ros::Publisher pub = n.advertise<geometry_msgs::Wrench>("/command", 100);
    bool ok=true;
    float force=atof(argv[1]);
    geometry_msgs::Wrench control;
    control.force.z=0;
    control.torque.x=0;
    control.torque.y=0;
    bool flick=false;
    while(ros::ok()&&ok)
    {
        int c = getch(); 
        if (c == 'w')
        {
            control.force.x=0;
            control.force.y=force;
        }
        else if (c == 's')
        {
            control.force.x=0;
            control.force.y=-force;
        }
        else if (c == 'a')
        {
            control.force.x=-force;
            control.force.y=0;
        }
        else if (c == 'd')
        {
            control.force.x=force;
            control.force.y=0;
        }
        else if (c == 'z')
        {
            control.force.x=0;
            control.force.y=0;
            control.torque.z=force;
        }
        else if (c == 'x')
        {
            control.force.x=0;
            control.force.y=0;
            control.torque.z=-force;
        }
        else if(c=='q')
        {
            control.force.x=0;
            control.force.y=0;
            pub.publish(control);
            ok=false;
        }
        else if(c=='f')
        {
            if(flick==false)
                flick=true;
            else
                flick=false;
        }
        pub.publish(control);
        if(!flick)
        {
            ros::Duration(0.02).sleep();
            control.force.x=0;
            control.force.y=0;
            control.torque.z=0;
            pub.publish(control);
        }
        ros::spinOnce();
    }
    
    return 0;
}