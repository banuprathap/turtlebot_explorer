/*
 *MIT License
 *
 *  Copyright (c) 2017 Banuprathap Anandan
 *
 *  AUTHOR : BANUPRATHAP ANANDAN
 *  AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
 *  EMAIL : BPRATHAP@UMD.EDU
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *
 *  Project: Frontier Explorer for Turtlebot
 *
 *
 */

/**
 * @file main.cpp
 * @brief
 * @author Banuprathap Anandan
 * @date   04/26/2017
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "wavefront_detection.hpp"
/**
 * @brief      Class for exploration task.
 */
class Explorer {
 public:
    /**
     * @brief      Default constructor to initiate publisher and subscriber
     *
     *
     * @param      nh    ROS NodeHandle
     */
    Explorer(ros::NodeHandle& nh);
    /**
     * @brief      Callback function from the map subscriber
     *
     * @param[in]  map   The map
     */
    void mapCallback(const nav_msgs::OccupancyGrid& map);
 protected:
    sensor_msgs::PointCloud frontier_cloud;
    ros::Publisher frontier_pub;
    ros::Subscriber mapSub;
};

/**
 * @brief      Execution starts here
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     Returns 0 upon successful execution
 */
int main(int argc, char **argv) {
  // Implement me
  return 0;
}
