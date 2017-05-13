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
 * @file navigator.hpp
 * @brief
 * @author Banuprathap Anandan
 * @date   05/07/2017
 */
#ifndef INCLUDE_TURTLEBOT_EXPLORER_NAVIGATOR_HPP_
#define INCLUDE_TURTLEBOT_EXPLORER_NAVIGATOR_HPP_
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;

/**
 * @brief      Class for navigator functionalities.
 */
class Navigator {
 public:
    /**
     * @brief      Default constructor
     *
     * @param      nh    ROS NodeHandle
     * @param      list  tfListener
     */
    Navigator(ros::NodeHandle& nh, tf::TransformListener& list);
    /**
     * @brief      Callback function for frontier subscriber
     *
     * @param[in]  frontier_cloud  The frontier cloud
     */
    void frontierCallback(const sensor_msgs::PointCloud frontier_cloud);
    /**
     * @brief      Returns the distance between two points
     *
     * @param[in]  x1    The x coordinate of point 1
     * @param[in]  x2    The x coordinate of point 2
     * @param[in]  y1    The y coordinate of point 1
     * @param[in]  y2    The y coordinate of point 2
     *
     * @return     The distance.
     */
    float getDistance(float x1, float x2, float y1, float y2);
    /**
     * @brief      Function to handle ROS spins
     */
    void spin();

 protected:
    /**
     * @brief      Returns the nearest frontier.
     *
     * @param[in]  frontier_cloud  The frontier cloud
     *
     * @return     The nearest frontier index.
     */
    int getNearestFrontier(const sensor_msgs::PointCloud frontier_cloud,
                           const tf::StampedTransform transform);
    /**
     * @brief      Fucntion to move the robot to
     *             the given map index
     *
     * @param[in]  x     x coordinate in map
     * @param[in]  y     y coordinate in map
     */
    void movetoFrontier(int x, int y);
    /**
     * Subscriber to the frontier publisher
     */
    ros::Subscriber frontierSub;
    /**
     * Transform Listener
     */
    tf::TransformListener *tfListener;
};

#endif  //  INCLUDE_TURTLEBOT_EXPLORER_NAVIGATOR_HPP_
