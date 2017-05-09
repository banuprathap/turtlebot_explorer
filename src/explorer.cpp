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
#include "explorer.hpp"


void Explorer::Explorer(ros::NodeHandle& nh) {
  frontier_pub = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
  mapSub = nh.subscribe("map", 1, &Explorer::mapCallback, this);
  frontier_cloud.header.frame_id = "map";
  Wavefront wf;
}

void Explorer::mapCallback(const nav_msgs::OccupancyGrid& map) {
  float resolution = map.info.resolution;
  float map_x = map.info.origin.position.x / resolution;
  float map_y = map.info.origin.position.y / resolution;
  float x = 0. - map_x;
  float y = 0. - map_y;
  std::vector<std::vector<int>> frontiers = wf.wfd(map, map.info.height,
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
      frontier_cloud.points[pointI].x = ((frontiers[i][j] % map.info.width) + map_x) *
                                        resolution;
      frontier_cloud.points[pointI].y = ((frontiers[i][j] / map.info.width) + map_y) *
                                        resolution;
      frontier_cloud.points[pointI].z = 0;
      pointI++;
    }
  }
  frontier_publisher.publish(frontier_cloud);
}