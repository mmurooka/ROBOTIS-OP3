/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: Masaki Murooka */

#include <stdio.h>
#include "op3_topic_control_module/topic_control_module.h"

namespace robotis_op
{

  TopicControlModule::TopicControlModule()
  {
    enable_ = false;
    module_name_ = "topic_control_module";
    control_mode_ = robotis_framework::PositionControl;
  }

  TopicControlModule::~TopicControlModule()
  {
    queue_thread_.join();
  }

  void TopicControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
  {
    // init result, joint_id_table
    int joint_index = 0;
    for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
	 it != robot->dxls_.end(); it++)
      {
	std::string joint_name = it->first;
	robotis_framework::Dynamixel* dxl_info = it->second;

	result_[joint_name] = new robotis_framework::DynamixelState();
	result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
	result_[joint_name]->goal_velocity_ = dxl_info->dxl_state_->goal_velocity_;
	joint_index_[joint_name] = joint_index++;
      }

    goal_position_.resize(result_.size());
    goal_velocity_.resize(result_.size());
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
	 state_it != result_.end(); state_it++) {
      std::string joint_name = state_it->first;
      int index = joint_index_[joint_name];
      goal_position_[index] = result_[joint_name]->goal_position_;
      goal_velocity_[index] = result_[joint_name]->goal_velocity_;
    }

    ros::NodeHandle ros_node;
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&TopicControlModule::queueThread, this));
  }

  void TopicControlModule::queueThread()
  {
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;
    ros_node.setCallbackQueue(&callback_queue);
    ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/topic_control/set_joint_states", 1,
							    &TopicControlModule::setJointCallback, this);
    ros::WallDuration duration(control_cycle_msec_ / 1000.0);
    ROS_INFO("[TopicControlModule] start callback queue thread loop.");
    while(ros_node.ok()) {
      callback_queue.callAvailable(duration);
    }
  }

  void TopicControlModule::setJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    for (int ix = 0; ix < msg->name.size(); ix++) {
      std::string joint_name = msg->name[ix];
      std::map<std::string, int>::iterator joint_it = joint_index_.find(joint_name);
      if (joint_it != joint_index_.end()) {
	int joint_index = joint_it->second;
	goal_position_[joint_index] = msg->position[ix];
	if (msg->position.size() == msg->velocity.size()) {
	  goal_velocity_[joint_index] = msg->velocity[ix];
	}
      }
    }
  }

  void TopicControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
				   std::map<std::string, double> sensors)
  {
    if (enable_ == false) {
      return;
    }

    // set joint data to robot
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
	 state_it != result_.end(); state_it++) {
      std::string joint_name = state_it->first;
      int index = joint_index_[joint_name];
      result_[joint_name]->goal_position_ = goal_position_[index];
      result_[joint_name]->goal_velocity_ = goal_velocity_[index];
    }
  }

  void TopicControlModule::stop()
  {
    return;
  }

  bool TopicControlModule::isRunning()
  {
    return false;
  }

}
