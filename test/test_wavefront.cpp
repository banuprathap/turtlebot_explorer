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
 *  Project: ROS Beginner Tutorial
 *
 *
 */
/**
 * @file test_explorer.cpp
 * @brief A simple unit test for explorer.cpp
 * @author Banuprathap Anandan
 * @date   05/12/2017
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "turtlebot_explorer/wavefront_detection.hpp"

Wavefront wf;
/**
 * @brief      test getNeighbours()
 *
 */
TEST(TESTSuite, getNeighbours) {
  int neighbours[8];
  wf.getNeighbours(neighbours, 55, 20);
  EXPECT_EQ(neighbours[0], 34);
  EXPECT_EQ(neighbours[1], 35);
  EXPECT_EQ(neighbours[2], 36);
  EXPECT_EQ(neighbours[3], 54);
  EXPECT_EQ(neighbours[4], 56);
  EXPECT_EQ(neighbours[5], 74);
  EXPECT_EQ(neighbours[6], 75);
  EXPECT_EQ(neighbours[7], 76);
}

/**
 * @brief      test isFrontierPoint()
 * 
 */

TEST(TESTSuite, isFrontierPoint) {
  nav_msgs::OccupancyGrid map;
  bool test(wf.isFrontierPoint(map, 10, 4000, 20));
  EXPECT_FALSE(test);
}
/**
 * @brief      program entrypoint
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     integer 0 upon exit success \n
 *            integer -1 upon exit failure
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "wavefront");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
