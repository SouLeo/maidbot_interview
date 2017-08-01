// Author: Selma Wanna
// Reference Material: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors#Writing_Code_to_Publish_a_LaserScan_Message


#include "lidar/laser_pub.h"

LaserScan::LaserScan() :
	n("~"),

	num_readings(100),
    laser_frequency(40),
	count(0),
	r(1.0)


{
	ros::Publisher laser_scan_pub = n.advertise<sensor_msgs::LaserScan>("sensor_msgs/LaserScan",50);

}

unsigned int LaserScan::get_num_readings(void) { return this->num_readings; }
double LaserScan::get_laser_frequency(void) { return this->laser_frequency; }
int LaserScan::get_count(void) { return this->count; }

int main(int argc, char **argv) {
	// ROS setup
	ros::init(argc, argv, "laser_pub");
	
	//create instance of LaserScan
	LaserScan* laser_pub_node = new LaserScan();

	while(ros::ok()){
		// generating fake data for laser scan
		
		for (unsigned int i = 0; i < laser_pub_node->get_num_readings(); i++){
		    laser_pub_node->ranges.push_back(laser_pub_node->get_count());
			laser_pub_node->intensities.push_back(100 + laser_pub_node->get_count());
		}
		ros::Time scan_time = ros::Time::now();
		
	}
	

	ros::waitForShutdown();

	return 0;
}
