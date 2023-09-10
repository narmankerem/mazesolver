#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

float sensor_left;
float sensor_right;
float sensor_front;
//float odom_x;
//float odom_y;

ros::Publisher cmd_vel_pub;

/*simulation

void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_front = msg->ranges[0];
    ROS_INFO("Front: %f", sensor_front);
}

void leftCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_left = msg->ranges[0];
    ROS_INFO("Left: %f", sensor_left);
}

void rightCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_right = msg->ranges[0];
    ROS_INFO("Right: %f", sensor_right);
}
*/

//Arduino
void frontCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Front: [%f]", msg->data);
  sensor_front = msg->data;
}


void rightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Right: [%f]", msg->data);
  sensor_right = msg->data;
}


void leftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Left: [%f]", msg->data);
  sensor_left = msg->data;
}

/*

void odomxCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("x: [%f]", msg->data);
  odom_x = msg->data;
}


void odomyCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("y: [%f]", msg->data);
  odom_y = msg->data;
}

*/



void move(double linear_vel, double angular_vel) {
	geometry_msgs::Twist cmd;
	cmd.linear.x = linear_vel;
   	cmd.angular.z = angular_vel;
   	cmd_vel_pub.publish(cmd);
}
    

int main(int argc, char** argv) {
    ros::init(argc, argv, "maze_solver_node");
    
    ros::NodeHandle n;	
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    /*Simulation
    ros::Subscriber sub_front = n.subscribe("/base_scan_1", 1000, frontCallback);
    ros::Subscriber sub_left = n.subscribe("/base_scan_2", 1000, leftCallback);
    ros::Subscriber sub_right = n.subscribe("/base_scan_3", 1000, rightCallback);
    */
     
    
    // Arduino
    ros::Subscriber sub_front = n.subscribe("/front_distance", 1000, frontCallback);
    ros::Subscriber sub_left = n.subscribe("/left_distance", 1000, leftCallback);
    ros::Subscriber sub_right = n.subscribe("/right_distance", 1000, rightCallback);
    //ros::Subscriber sub_x = n.subscribe("/odom_x", 1000, odomxCallback);
    //ros::Subscriber sub_y= n.subscribe("/odom_y", 1000, odomyCallback);
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
       	
        if (sensor_left >= 250) {
            ROS_INFO("lost");
            move(0.06,0.30);
        } else if (sensor_front >= 400 && sensor_left >= 100 && sensor_left <= 250) {
            ROS_INFO("forward");
            move(0.1,0.0); 
        } else if (sensor_front <= 400 || sensor_left <= 100) {
            ROS_INFO("right");
            move(0.05,-1.0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

