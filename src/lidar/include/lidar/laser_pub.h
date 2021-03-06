#ifndef _LASER_PUB_H_
#define _LASER_PUB_H_ 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

class LaserScan {
    private:
        unsigned int num_readings;
        double laser_frequency;
        ros::NodeHandle n;

    public:
    // ROS
        ros::Publisher laser_scan_pub;
        
        // Filling Scan Messages
        sensor_msgs::LaserScan scan;
        sensor_msgs::LaserScan * fill_scan(sensor_msgs::LaserScan& scan, ros::Time scan_time, std::string frame_id, float ang_min, float ang_max, float ang_incr, double time_incr, float range_min, float range_max, unsigned int num_readings);
        
        // Member access
        std::vector<double> ranges;
        std::vector<double> intensities;

        unsigned int get_num_readings();
        double get_laser_frequency();
		
        // constructor/destructor
        LaserScan();
        ~LaserScan();
};

#endif // _LASER_PUB_H_

