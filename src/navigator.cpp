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
 * @file navigator.cpp
 * @brief
 * @author Banuprathap Anandan
 * @date   05/07/2017
 */
#include <cstdlib>  //  Needed for rand()
#include <ctime>  //  Needed to seed random number generator with a time value
#include "turtlebot_explorer/navigator.hpp"

Navigator::Navigator(ros::NodeHandle& nh, tf::TransformListener& list) {
  //  srand(time(NULL));
  tfListener = &list;
  frontierSub = nh.subscribe("frontiers", 1,
                             &Navigator::frontierCallback, this);
}

float Navigator::getDistance(float x1, float x2, float y1, float y2) {
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

void Navigator::frontierCallback(const sensor_msgs::PointCloud frontier_cloud) {
  tf::StampedTransform transform;
  tfListener->waitForTransform("/map", "/odom",
                               ros::Time(0), ros::Duration(3.0));
  tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
  if (frontier_cloud.points.size() == 0)
    return;
  int frontier_i = getNearestFrontier(frontier_cloud, transform);
  ROS_INFO("Closest frontier: %d", frontier_i);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  bool at_target = false;
  int attempts = 0;
  while (!at_target && attempts < 2) {
    if (attempts >= 0) {
      frontier_i = (rand() % frontier_cloud.points.size());
      at_target = false;
    }
    attempts++;
    goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
    goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server active");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(45.0));
    ROS_INFO("move_base goal published");
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      at_target = true;
      ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(3.14);
      goal.target_pose.pose.orientation = odom_quat;
      ac.sendGoal(goal);
      ac.waitForResult();
    }
  }
}

int Navigator::getNearestFrontier(const sensor_msgs::PointCloud
      frontier_cloud, const tf::StampedTransform transform) {
  int frontier_i = 0;
  float closest_frontier_distance = 100000;
  for (int i = 0; i < frontier_cloud.points.size(); i++) {
    float distance = getDistance(frontier_cloud.points[i].x,
                                 transform.getOrigin().x(),
                                 frontier_cloud.points[i].y,
                                 transform.getOrigin().y());
    if (distance > .7 && distance <= closest_frontier_distance) {
      closest_frontier_distance = distance;
      frontier_i = i;
    }
  }
  return frontier_i;
}

void Navigator::spin() {
  ros::Rate rate(50);  //  Specify the FSM loop rate in Hz
  while (ros::ok()) {  //  Keep spinning loop until user presses Ctrl+C
    ros::spinOnce();  //  Allow ROS to process incoming messages
    rate.sleep();  //  Sleep for the rest of the cycle
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "TurtlebotNavigation");  //  Initiate new ROS node
  ros::NodeHandle n;
  tf::TransformListener listener;
  Navigator walker(n, listener);
  ROS_INFO("INFO! Navigation Started");
  walker.spin();  //  Execute FSM loop
  return 0;
}
