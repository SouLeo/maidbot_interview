# MaidBot Interview Take Home Project

### Author: Selma Wanna
### email: slwanna@utexas.edu

## Table of Contents
1. [About](#about)
2. [Installation](#installation)
3. [How to Run](#how-to-run)

## About
This project will launch an odometry node which will move an imaginary robot in
a circle. As the robot moves in a circle, it will capture fake lidar data. Rviz
will be launched along with the fake odometry and lidar nodes to visualize the
concept. 

## Installation
1. Navigate to your home directory and type
    ```
    git clone https://github.com/SouLeo/maidbot_interview.git
    ```
2. Compile the packages
    ```
    cd ~/maidbot_interview
    catkin build
    ```

3. Adjust your bashrc
    ```
    cd ~/
    vi ~/.bashrc
    ```
   Write the following at the end of the file
   ```
   source ~/maidbot_interview/devel/setup.bash
   ```

## How to Run<a name="how-to-run" />
1. Make sure your environment is properly sourced
    ```
    source ~/maidbot_interview/devel/setup.bash
    ```
2. Launch the full system
    ```
    roslaunch full_sys full_sys.launch
    ```
