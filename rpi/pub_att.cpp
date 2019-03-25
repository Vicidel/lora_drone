#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
 
//#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;
 
   ros::Publisher pub_att = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
   ros::Publisher pub_thr = n.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 100);
   ros::Rate loop_rate(100);
   ros::spinOnce();
   geometry_msgs::PoseStamped cmd_att;
   std_msgs::Float64 cmd_thr;
   int count = 1;
   double v[3]={1.0, 0.0, 0.0};
   double lambda = 0.3;
 
 
   double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
   double theta=0.0;
    
 
        //PositionReciever qp;
        //Body some_object;
        //qp.connect_to_server();
 
     
   while(ros::ok()){
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
        //Create attitude command message
       cmd_att.header.stamp = ros::Time::now();
       cmd_att.header.seq=count;
       cmd_att.header.frame_id = 1;
       cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
       cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
       cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;
 
       cmd_att.pose.orientation.x = sin(theta/2.0)*v[0]/v_norm;
       cmd_att.pose.orientation.y = sin(theta/2.0)*v[1]/v_norm;
       cmd_att.pose.orientation.z = sin(theta/2.0)*v[2]/v_norm;
       cmd_att.pose.orientation.w = cos(theta/2.0);
      /*
       double q_norm = sqrt(sin(theta/2.0)*sin(theta/2.0)+cos(theta/2.0)*cos(theta/2.0));
        printf("%f\t",v_norm);
        printf("%f\n", q_norm);
    */
       //Create throttle command message
       cmd_thr.data = lambda;
       
       pub_att.publish(cmd_att);
       pub_thr.publish(cmd_thr);
       ros::spinOnce();
       count++;
       theta=0.3*sin(count/300.0);
       loop_rate.sleep();
/*
    if(count>1000){
        count =0;
    }
    */
   }
    
       
   return 0;
}