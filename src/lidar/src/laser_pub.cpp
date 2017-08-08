// Author: Selma Wanna 
// Reference Material:
// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors#Writing_Code_to_Publish_a_LaserScan_Message
// http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

#include "lidar/laser_pub.h"

LaserScan::LaserScan() : n("~"), num_readings(100), laser_frequency(40)

{ laser_scan_pub =
    n.advertise<sensor_msgs::LaserScan>("sensor_msgs/LaserScan",50);
}

// Getters for private member variables

unsigned int LaserScan::get_num_readings(void) { return this->num_readings; }

double LaserScan::get_laser_frequency(void) { return this->laser_frequency; }

// Fill Laser Scan Message function
void fill_scan(sensor_msgs::LaserScan& scan, std::string
        frame_id, float ang_min, float ang_max, float ang_incr, double
        time_incr, float range_min, float range_max, unsigned int num_readings) {
    scan.header.frame_id = frame_id;
    scan.angle_min = ang_min;
    scan.angle_max = ang_max;
    scan.angle_increment = ang_incr;
    scan.time_increment = time_incr;
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
}

int main(int argc, char **argv) {
    unsigned int count(0);
    // ROS setup
    ros::init(argc, argv, "laser_pub");

    LaserScan* laser_pub_node = new LaserScan; 

    ros::Rate r(1.0);

    float ang_incr = 3.14 / laser_pub_node->get_num_readings(); 
    double time_incr = (1 / laser_pub_node->get_laser_frequency()) /
        laser_pub_node->get_num_readings();

    fill_scan(laser_pub_node->scan, "laser_frame", -1.57, 1.57,
            ang_incr, time_incr, 0.00, 100.0,
            laser_pub_node->get_num_readings());
    
    while(ros::ok()){
    
        // generating fake data for laser scan
        for (unsigned int i = 0; i < laser_pub_node->get_num_readings(); i++){
            laser_pub_node->ranges.push_back(count);
            laser_pub_node->intensities.push_back(100 + count); 
        } 
        
        //populate LaserScan message
        ros::Time scan_time = ros::Time::now();
        laser_pub_node->scan.header.stamp = scan_time; 
        for(unsigned int i = 0; i < laser_pub_node->get_num_readings(); i++){
            laser_pub_node->scan.ranges[i] = laser_pub_node->ranges[i];
            laser_pub_node->scan.intensities[i] = laser_pub_node->intensities[i];
        }

        laser_pub_node->ranges.clear();
        laser_pub_node->intensities.clear(); 

        laser_pub_node->laser_scan_pub.publish(laser_pub_node->scan); 
        count = rand() % 10;
//        ROS_INFO("count: %d", count);
        r.sleep();
    }	

    ros::waitForShutdown();

    return 0; 
}
