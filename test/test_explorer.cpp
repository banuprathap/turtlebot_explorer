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
#include "turtlebot_explorer/explorer.hpp"



/**
 * @brief      test mapCallback function
 *
 *             No testable feature, check if function call works
 *
 */
TEST(TESTSuite, mapcallback) {
  ros::NodeHandle nh;
  Explorer explore(nh);
  nav_msgs::OccupancyGrid map;
  explore.mapCallback(map);
  EXPECT_TRUE(true);
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
  ros::init(argc, argv, "explorer");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
