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
 * @file wavefront_detection.cpp
 * @brief
 * @author Banuprathap Anandan
 * @date   05/07/2017
 */
#include "wavefront_detection.hpp"
#define OCC_THRESH 10  //  threshold value to determine cell occupancy
/**
 * @brief      Returns the neighbours of a cell
 *
 * @param      n_array    The array containing neighbours
 * @param[in]  pos        The position of current cell
 * @param[in]  map_width  The map width
 */
void wavefront::get_neighbours(int &n_array[], int pos, int map_width) {
  n_array[0] = pos - map_width - 1;
  n_array[1] = pos - map_width;
  n_array[2] = pos - map_width + 1;
  n_array[3] = pos - 1;
  n_array[4] = pos + 1;
  n_array[5] = pos + map_width - 1;
  n_array[6] = pos + map_width;
  n_array[7] = pos + map_width + 1;
}
/**
 * @brief      Determines if a given point is a frontier point.
 *
 * @param[in]  map        The map
 * @param[in]  point      The point of interest
 * @param[in]  map_size   The map size
 * @param[in]  map_width  The map width
 *
 * @return     True if it's a frontier point, False otherwise.
 */
bool wavefront::is_frontier_point(const nav_msgs::OccupancyGrid& map, int point,
                                  int map_size, int map_width) {
  int neighbours[8];
  get_neighbours(neighbours, point, map_width);
  bool found(false);
  for (int i = 0; i < 8; i++) {
    if (neighbours[i] >= 0 && neighbours[i] < map_size) {
      if (map.data[neighbours[i]] == 0) {
        found = true;
      }
    }
  }
  return found;
}
/**
 * @brief      Wavefront Frontier Detector
 *
 * @param[in]  map         The map
 * @param[in]  map_height  The map height
 * @param[in]  map_width   The map width
 * @param[in]  pose        Current pose of robot
 *
 * @return     Returns a list of frontiers
 */
std::vector<std::vector<int>> wfd(const nav_msgs::OccupancyGrid& map,
int map_height, int map_width, int pose) {
  std::vector<std::vector<int>> frontiers;
}
