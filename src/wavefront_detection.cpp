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
#define MAP_OPEN_LIST 1
#define MAP_CLOSE_LIST 2
#define FRONTIER_OPEN_LIST 3
#define FRONTIER_CLOSE_LIST 4
/**
 * @brief      Returns the neighbours of a cell
 *
 * @param      n_array    The array containing neighbours
 * @param[in]  pos        The position of current cell
 * @param[in]  map_width  The map width
 */
void Wavefront::getNeighbours(int &n_array[], int pos, int map_width) {
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
bool Wavefront::isFrontierPoint(const nav_msgs::OccupancyGrid& map, int point,
                                  int map_size, int map_width) {
  if (map.data[point] != -1) {
    return false;
  }
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
std::vector<std::vector<int>> Wavefront::wfd(const nav_msgs::OccupancyGrid& map,
int map_height, int map_width, int pose) {
  std::vector<std::vector<int>> frontiers;
  int map_size = map_height * map_width;
  std::map<int, int> cell_states;
  queue<int> q_m;
  q_m.push(pose);
  cell_states[pose] = MAP_OPEN_LIST;
  int adj_vector[8];
  int v_neighbours[8];
  while (!q_m.empty()) {
    int cur_pos = q_m.front();
    q_m.pop();
    ROS_INFO("cur_pos: %d, cell_state: %d", cur_pos, cell_states[cur_pos]);
    if (cell_states[cur_pos] == MAP_CLOSE_LIST)
      continue;
    if (is_frontier_point(map, cur_pos, map_size, map_width)) {
      std::queue<int> q_f;
      std::vector<int> new_frontier;
      q_f.push(cur_pos);
      cell_states[cur_pos] = FRONTIER_OPEN_LIST;
      //  Second BFS
      while (!q_f.empty()) {
        ROS_INFO("Size: %d", q_f.size());
        int n_cell = q_f.front();
        q_f.pop();
        if (cell_states[n_cell] == MAP_CLOSE_LIST
            || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
          continue;
        if (is_frontier_point(map, n_cell, map_size, map_width)) {
          ROS_INFO("adding %d to frontiers", n_cell);
          new_frontier.push_back(n_cell);
          get_neighbours(adj_vector, cur_pos, map_width);
          for (int i = 0; i < 8; i++) {
            if (adj_vector[i] < map_size && adj_vector[i] >= 0) {
              if (cell_states[adj_vector[i]] != FRONTIER_OPEN_LIST &&
                  cell_states[adj_vector[i]] != FRONTIER_CLOSE_LIST &&
                  cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
                if (map.data[adj_vector[i]] != 100) {
                  q_f.push(adj_vector[i]);
                  cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST;
                }
              }
            }
          }
        }
        cell_states[n_cell] = FRONTIER_CLOSE_LIST;
      }
      if (new_frontier.size() > 2)
        frontiers.push_back(new_frontier);
      for (unsigned int i = 0; i < new_frontier.size(); i++) {
        cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
      }
    }
    get_neighbours(adj_vector, cur_pos, map_width);
    for (int i = 0; i < 8; ++i) {
      if (adj_vector[i] < map_size && adj_vector[i] >= 0) {
        if (cell_states[adj_vector[i]] != MAP_OPEN_LIST
            &&  cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
          get_neighbours(v_neighbours, adj_vector[i], map_width);
          bool map_open_neighbor = false;
          for (int j = 0; j < 8; j++) {
            if (v_neighbours[j] < map_size && v_neighbours[j] >= 0) {
              if (map.data[v_neighbours[j]] < OCC_THRESHOLD
                  && map.data[v_neighbours[j]] >= 0) {  //  >= 0 AANPASSING
                map_open_neighbor = true;
                break;
              }
            }
          }
          if (map_open_neighbor) {
            q_m.push(adj_vector[i]);
            cell_states[adj_vector[i]] = MAP_OPEN_LIST;
          }
        }
      }
    }
    cell_states[cur_pos] = MAP_CLOSE_LIST;
  }
  return frontiers;
}

