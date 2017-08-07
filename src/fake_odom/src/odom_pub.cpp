// Author: Selma Wanna
// Reference Material:
// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "fake_odom/odom_pub.h"

FakeOdom::FakeOdom() : n("~"), vx(0.1), vy(-0.1), vth(0.1)
{   
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    x = 0.0;
    y = 0.0;
    th = 0.0;
}

float FakeOdom::getvx(void){ return this->vx; }

float FakeOdom::getvy(void){ return this->vy; }

float FakeOdom::getvth(void){ return this->vth; }

void FakeOdom::odom_trans_init(geometry_msgs::TransformStamped& foo, ros::Time current){
    foo.header.stamp = current;
    foo.header.frame_id = "odom";
    foo.child_frame_id = "base_link";

    foo.transform.translation.z = 0.0;
}

void FakeOdom::odom_trans_update(geometry_msgs::TransformStamped& foo, ros::Time current, double x, double y, geometry_msgs::Quaternion odom_quat){
    foo.header.stamp = current;
    foo.transform.translation.x = x;
    foo.transform.translation.y = y;
    foo.transform.rotation = odom_quat;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_pub");

    FakeOdom* odom_node = new FakeOdom;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);

    odom_node->odom_trans_init(odom_node->odom_trans, current_time);

    while(ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();

        // compute odom 
        odom_node->dt = (current_time - last_time).toSec();
        odom_node->delta_x = (odom_node->getvx() * cos(odom_node->th) - odom_node->getvy() * sin(odom_node->th)) * odom_node->dt;
        odom_node->delta_y = (odom_node->getvx() * sin(odom_node->th) + odom_node->getvy() * cos(odom_node->th)) * odom_node->dt;
        odom_node->delta_th = odom_node->getvth() * odom_node->dt;
    
        odom_node->x +=  odom_node->delta_x;
        odom_node->y +=  odom_node->delta_y;
        odom_node->th +=  odom_node->delta_th;

        odom_node->odom_quat = tf::createQuaternionMsgFromYaw(odom_node->th);
        odom_node->odom_trans_update(odom_node->odom_trans, current_time, x, y, odom_quat);
        
        // send the transform
        odom_node->odom_broadcaster.sendTransform(odom_trans);

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
}
