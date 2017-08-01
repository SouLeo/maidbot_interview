#ifndef _LASER_PUB_H_
#define _LASER_PUB_H_ 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScan {
    private:
		unsigned int num_readings;
		double laser_frequency;
        int count; 
		ros::Rate r;
	    sensor_msgs::LaserScan scan;

		ros::NodeHandle n;
		ros::Publisher laser_scan_pub;

	public:
		// member access

		std::vector<double> ranges;
		std::vector<double> intensities;

		unsigned int get_num_readings();
		double get_laser_frequency();
		int get_count();
		



		// constructor/destructor
		
		LaserScan();
		~LaserScan();


};

#endif // _LASER_PUB_H_

