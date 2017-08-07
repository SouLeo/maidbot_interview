// Author: Selma Wanna
// Reference Material:
// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "fake_odom/odom_pub.h"

FakeOdom::FakeOdom() : n("~"), vx(0.1), vy(-0.1), vth(0.1)
{   
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    x(0.0);
    y(0.0);
    th(0.0);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_pub");

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);
    while(ros.ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();

        // compute odom 
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
    
        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // pub the odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }

