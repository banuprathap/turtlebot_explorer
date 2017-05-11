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
 * @file wavefront_detection.hpp
 * @brief
 * @author Banuprathap Anandan
 * @date   05/06/2017
 */
#ifndef SRC_WAVEFRONT_DETECTION_HPP_
#define SRC_WAVEFRONT_DETECTION_HPP_
#include <vector>
#include <queue>
#include <map>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


class Wavefront {
 public:
    void getNeighbours(int& n_array[], int position, int map_width);
    bool isFrontierPoint(const nav_msgs::OccupancyGrid& map, int point,
                           int map_size, int map_width);
    std::vector<std::vector<int>> wfd(const nav_msgs::OccupancyGrid& map,
                                      int map_height, int map_width, int pose);
};

#endif  //  SRC_WAVEFRONT_DETECTION_HPP_
