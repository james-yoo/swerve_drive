/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */
/* Modified by Kyungho Yoo (James) */

#ifndef SWERVEDRIVE_CONTROL_H
#define SWERVEDRIVE_CONTROL_H

#include <ros/ros.h>

#include "message_header.h"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <geometry_msgs/Twist.h>

class SwerveDriveControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;

  // ROS Topic Subscriber
  ros::Subscriber twist_msgs_sub_;

  // ROS Service Server
  ros::ServiceServer swervedrive_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[2];
  uint8_t dxl_cnt_;

 public:
  SwerveDriveControl();
  ~SwerveDriveControl();
  void controlLoop(void);

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();

  void swervedriveTwistMsgCallback(const geometry_msgs::Twist::ConstPtr& msg); 
};

#endif //SWERVEDRIVE_CONTROL_H
