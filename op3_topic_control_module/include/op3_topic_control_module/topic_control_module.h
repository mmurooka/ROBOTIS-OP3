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

#ifndef TOPIC_CONTROL_MODULE_H_
#define TOPIC_CONTROL_MODULE_H_

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace robotis_op
{

  class TopicControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<TopicControlModule>
  {
  public:
    TopicControlModule();
    ~TopicControlModule();

    void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
    void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

    void stop();
    bool isRunning();

    void onModuleEnable() {};
    void onModuleDisable() {};

  private:
    /* ROS Topic Callback Functions */
    void setJointCallback(const sensor_msgs::JointState::ConstPtr &msg);

    void queueThread();

    double control_cycle_msec_;
    boost::thread queue_thread_;

    std::vector<double> goal_position_;
    std::vector<double> goal_velocity_;

    std::map<std::string, int> joint_index_;
  };

}

#endif /* TOPIC_CONTROL_MODULE_H_ */
