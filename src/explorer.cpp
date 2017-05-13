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
 * @file explorer.cpp
 * @brief
 * @author Banuprathap Anandan
 * @date   05/07/2017
 */
#include <vector>
#include <cstdlib>  //  Needed for rand()
#include <ctime>  //  Needed to seed random number generator with a time value
#include "turtlebot_explorer/explorer.hpp"

Explorer::Explorer(ros::NodeHandle& nh) {
  //  srand(time(NULL));
  frontier_pub = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
  mapSub = nh.subscribe("map", 1, &Explorer::mapCallback, this);
  frontier_cloud.header.frame_id = "map";
}

void Explorer::mapCallback(const nav_msgs::OccupancyGrid& map) {
  Wavefront wtf;
  float resolution = map.info.resolution;
  float map_x = map.info.origin.position.x / resolution;
  float map_y = map.info.origin.position.y / resolution;
  float x = 0. - map_x;
  float y = 0. - map_y;
  std::vector<std::vector<int> > frontiers = wtf.wfd(map, map.info.height,
      map.info.width, x + (y * map.info.width));
  int num_points = 0;
  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {
      num_points++;
    }
  }
  frontier_cloud.points.resize(num_points);
  int pointI = 0;
  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {
      frontier_cloud.points[pointI].x =
      ((frontiers[i][j] % map.info.width) + map_x) * resolution;
      frontier_cloud.points[pointI].y =
      ((frontiers[i][j] / map.info.width) + map_y) * resolution;
      frontier_cloud.points[pointI].z = 0;
      pointI++;
    }
  }
  frontier_pub.publish(frontier_cloud);
}

void Explorer::spin() {
  ros::Rate rate(10);  //  Specify the FSM loop rate in Hz
  while (ros::ok()) {  //  Keep spinning loop until user presses Ctrl+C
    ros::spinOnce();  //  Allow ROS to process incoming messages
    frontier_pub.publish(frontier_cloud);
    rate.sleep();  //  Sleep for the rest of the cycle
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv,
            "TurtlebotExploration");  //  Initiate new ROS node
  ros::NodeHandle n;
  Explorer explore(n);
  ROS_INFO("INFO! FRONTIERS");
  explore.spin();  //  Execute FSM loop
  return 0;
}
