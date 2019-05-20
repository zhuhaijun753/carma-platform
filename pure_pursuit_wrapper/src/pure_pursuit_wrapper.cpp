/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "pure_pursuit_wrapper/pure_pursuit_wrapper.hpp"

namespace pure_pursuit_wrapper {

PurePursuitWrapper::PurePursuitWrapper(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
  }

  ROS_INFO("Successfully launched node.");
}

PurePursuitWrapper::~PurePursuitWrapper()
{
}

bool PurePursuitWrapper::readParameters()
{
  // nodeHandle_.param<std::string>("SpeedAccelPublisher_topic", SpeedAccelPublisherTopic_, "/republish/cmd_speed");
  // nodeHandle_.param<std::string>("WrenchEffortPublisher_topic", WrenchEffortPublisherTopic_, "/republish/cmd_longitudinal_effort");
  // nodeHandle_.param<std::string>("LateralControlPublisher_topic", LateralControlPublisherTopic_, "/republish/cmd_lateral");
  // nodeHandle_.param("publish_rate", rate, 10);
  // nodeHandle_.param("timeout_thresh", timeout, 0.5);

  return true;
}

}