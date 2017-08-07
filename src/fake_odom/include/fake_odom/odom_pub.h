#ifndef _ODOM_PUB_H_
#define _ODOM_PUB_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class FakeOdom{
    private:
        // ROS
        ros::NodeHandle n;  

        // velocities
        double vx;
        double vy;
        double vth;
    
    public:
        // ROS
        ros::Publisher odom_pub;
        nav_msgs::Odometry odom;

        // velocities
        double x;
        double y;
        double z;

        // constructor/destructor
        FakeOdom();
        ~FakeOdom();
};

#endif //ODOM_PUB_H
