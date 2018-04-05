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

#include "swerve_drive/swerve_drive_control.h"

SwerveDriveControl::SwerveDriveControl()
    :node_handle_(""),
     dxl_cnt_(2)
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_id_[0] = node_handle_.param<int>("drive_wheel", 1);
  dxl_id_[1] = node_handle_.param<int>("steer_wheel", 2);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    uint16_t get_model_number;
    if (dxl_wb_->ping(dxl_id_[index], &get_model_number) != true)
    {
      ROS_ERROR("Not found Motors, Please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Velocity");

  initPublisher();
  initSubscriber();
}

SwerveDriveControl::~SwerveDriveControl()
{
  for (int index = 0; index < 2; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void SwerveDriveControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench_ur controller; swervedrive control example       \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void SwerveDriveControl::initPublisher()
{
  joint_state_list_pub_ = node_handle_.advertise<sensor_msgs::JointState>("swerve_drive/joint_states", 10);
}

void SwerveDriveControl::initSubscriber()
{
  twist_msgs_sub_ = node_handle_.subscribe("/cmd_vel", 10, &SwerveDriveControl::swervedriveTwistMsgCallback, this);
}

void SwerveDriveControl::jointStatePublish()
{
  sensor_msgs::JointState message;
  message.header.stamp = ros::Time::now();

  float position_radian = 0.0f;
  float velocity_radian = 0.0f;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    if( dxl_id_[index] == 1){
      message.name.push_back( "drive" );
    }
    else if( dxl_id_[index] == 2){
      message.name.push_back( "steer" );
    }
    else{
    message.name.push_back( std::string(dxl_wb_->getModelName(dxl_id_[index])) );
    }

    position_radian = dxl_wb_->convertValue2Radian(dxl_id_[index],dxl_wb_->itemRead(dxl_id_[index], "Present_Position") );
    velocity_radian = dxl_wb_->convertValue2Velocity(dxl_id_[index],dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity") );
    message.position.push_back( position_radian );
    message.velocity.push_back( velocity_radian );
  }

  joint_state_list_pub_.publish(message);
}

void SwerveDriveControl::controlLoop()
{
  jointStatePublish();
}

void SwerveDriveControl::swervedriveTwistMsgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  //Using the callback function just for subscribing  
  //Subscribing the message and storing it in 'linx' and 'angZ'
  //linx = msg.linear.x;      angZ = msg.angular.z;

  static int32_t goal_velocity[2] = {0, 0};
  float position_radian = 0.0f;
 
  goal_velocity[0] = dxl_wb_->convertVelocity2Value(dxl_id_[0], (-1) * msg->linear.x);
  goal_velocity[1] = dxl_wb_->convertVelocity2Value(dxl_id_[1], (-1) * msg->angular.z);
  
  bool ret = dxl_wb_->syncWrite("Goal_Velocity", goal_velocity);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "swervedrive_control");
  SwerveDriveControl swerve_ctrl;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    swerve_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
