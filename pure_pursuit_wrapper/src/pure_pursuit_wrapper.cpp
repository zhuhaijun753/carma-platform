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

PurePursuitWrapper::PurePursuitWrapper(ros::NodeHandle &nodeHandle): nh_(nodeHandle)
{
  if (!ReadParameters())
  {
    ROS_ERROR("Could not read parameters.");
  }

  Initialize();

  ROS_INFO("Successfully launched node.");
}

PurePursuitWrapper::~PurePursuitWrapper() {
}

void PurePursuitWrapper::Initialize() {
  
  // SystemAlert Subscriber
  system_alert_sub_ = nh_.subscribe("system_alert", 10, &PurePursuitWrapper::SystemAlertHandler, this);
  // SystemAlert Publisher
  system_alert_pub_ = nh_.advertise<cav_msgs::SystemAlert>("system_alert", 10, true);

  // SystemAlert Subscriber
  trajectory_plan_sub_ = nh_.subscribe("trajectory_plan", 10, &PurePursuitWrapper::TrajectoryPlanHandler, this);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);
  // CurrentPose Publisher
  // current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 10, true);
  // CurrentVelocity Publisher
  current_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("current_velocity", 10, true);
}

bool PurePursuitWrapper::ReadParameters() {
  // nodeHandle_.param<std::string>("SpeedAccelPublisher_topic", SpeedAccelPublisherTopic_, "/republish/cmd_speed");
  // nodeHandle_.param<std::string>("WrenchEffortPublisher_topic", WrenchEffortPublisherTopic_, "/republish/cmd_longitudinal_effort");
  // nodeHandle_.param<std::string>("LateralControlPublisher_topic", LateralControlPublisherTopic_, "/republish/cmd_lateral");
  // nodeHandle_.param("publish_rate", rate, 10);
  // nodeHandle_.param("timeout_thresh", timeout, 0.5);

  return true;
}

void PurePursuitWrapper::TrajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& msg){
    try {
      ROS_DEBUG_STREAM("Received TrajectoryPlan message");
      autoware_msgs::Lane lane;
      autoware_msgs::Waypoint waypoint;
      std::vector <autoware_msgs::Waypoint> waypoints; 
      waypoints.insert(waypoints.begin(), waypoint);
      lane.header = msg->header;
      // std::string::size_type sz;
      // lane.lane_id = std::stoi (msg->trajectory_id,&sz);
      lane.waypoints = waypoints;
      // lane.waypoints = waypoint;

      PublisherForWayPoints(lane);
    }
    catch(const std::exception& e) {
      HandleException(e);
    }

};

void PurePursuitWrapper::SystemAlertHandler(const cav_msgs::SystemAlert::ConstPtr& msg) {
    try {
      ROS_INFO_STREAM("Received SystemAlert message of type: " << msg->type);
      switch(msg->type) {
        case cav_msgs::SystemAlert::SHUTDOWN: 
          Shutdown(); // Shutdown this node when a SHUTDOWN request is received 
          break;
      }
    }
    catch(const std::exception& e) {
      HandleException(e);
    }
};

// void PurePursuitWrapper::PublisherForCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){
//   try {
//     current_pose_pub_.publish(msg);
//   }
//   catch(const std::exception& e) {
//     HandleException(e);
//   }
// };

void PurePursuitWrapper::PublisherForCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg){
  try {
    current_velocity_pub_.publish(msg);
  }
  catch(const std::exception& e) {
    HandleException(e);
  }
};

void PurePursuitWrapper::PublisherForWayPoints(autoware_msgs::Lane& msg){
  try {
    way_points_pub_.publish(msg);
  }
  catch(const std::exception& e) {
    HandleException(e);
  }
};


void PurePursuitWrapper::HandleException(const std::exception& e) {
  ROS_DEBUG("Sending SystemAlert Message");
  // Create system alert message
  cav_msgs::SystemAlert alert_msg;
  alert_msg.type = cav_msgs::SystemAlert::FATAL;
  alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
 
  ROS_ERROR_STREAM(alert_msg.description); // Log exception

  system_alert_pub_.publish(alert_msg); // Notify the rest of the system

  ros::Duration(0.05).sleep(); // Leave a small amount of time for the alert to be published
  Shutdown(); // Shutdown this node
}

void PurePursuitWrapper::Shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}

}  // namespace pure_pursuit_wrapper