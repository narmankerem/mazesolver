#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

class RobotDriver {
public:
    RobotDriver() {
        pose_sub = nh.subscribe("/pose", 10, &RobotDriver::poseCallback, this);
        laser_scan_sub = nh.subscribe("/laser_scan", 10, &RobotDriver::laserScanCallback, this);

        // Publish pose data on a different topic (/robot_pose) using geometry_msgs::Pose2D
        pose_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose", 10);
        laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/laser_scan_filtered", 10);

        tf_broadcaster = new tf2_ros::TransformBroadcaster();
    }

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
        // Republish the pose information on /robot_pose topic
        geometry_msgs::Pose2D robot_pose;
        robot_pose.x = pose_msg->x;
        robot_pose.y = pose_msg->y;
        robot_pose.theta = pose_msg->theta;

        pose_pub.publish(robot_pose);

        // Broadcast tf transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = robot_pose.x;
        transformStamped.transform.translation.y = robot_pose.y;
        transformStamped.transform.translation.z = 0.0;

        geometry_msgs::Quaternion odom_quat;
        odom_quat.z = sin(robot_pose.theta / 2.0);
        odom_quat.w = cos(robot_pose.theta / 2.0);
        transformStamped.transform.rotation = odom_quat;

        tf_broadcaster->sendTransform(transformStamped);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg) {
        sensor_msgs::LaserScan filtered_scan = *laser_scan_msg;
        filtered_scan.header.frame_id = "base_link";

        // You can apply filtering or other processing to the laser scan data here

        laser_scan_pub.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Subscriber laser_scan_sub;

    ros::Publisher pose_pub;  // Publish pose on a separate topic
    ros::Publisher laser_scan_pub;

    tf2_ros::TransformBroadcaster* tf_broadcaster;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_driver_node");
    RobotDriver driver;
    ros::spin();
    return 0;
}

