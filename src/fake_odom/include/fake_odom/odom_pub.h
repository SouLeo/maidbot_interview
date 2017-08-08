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
        geometry_msgs::Quaternion odom_quat;
        geometry_msgs::TransformStamped odom_trans;
        tf::TransformBroadcaster odom_broadcaster;

        // velocities
        double x;
        double y;
        double th;

        double dt;
        double delta_x;
        double delta_y;
        double delta_th;

        // member function
        void odom_trans_init(geometry_msgs::TransformStamped& foo, ros::Time current);
        void odom_trans_update(geometry_msgs::TransformStamped& foo, ros::Time current, double x, double y, geometry_msgs::Quaternion odom_quat);

        // getters
        float getvx();
        float getvy();
        float getvth();
        
        
        // constructor/destructor
        FakeOdom();
        ~FakeOdom();
};

#endif //ODOM_PUB_H
