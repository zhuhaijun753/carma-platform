/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <ros/ros.h>
#include <string>
#include "safe_stop.h"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include "route_end.h"
#include <cav_msgs/TrajectoryPlan.h>

namespace safe_stop_planner
{
    //safe_stop_planner::Safe_Stop();

    void Safe_Stop::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        guidance_state_sub_= nh_->subscribe("guidance_state", 1 , &Safe_Stop::guidance_state_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1 , &Safe_Stop::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1 , &Safe_Stop::twist_cb, this);

        wml_.reset(new carma_wm::WMListener());
        //set world model point from wm listener
        wm_ = wml_->getWorldModel();
        // ros::CARMANodeHandle::setSpinCallback([this])() -> bool
        // {
        //     return true;
        // }
        
    }

    void Safe_Stop::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void Safe_Stop::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void Safe_Stop::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_= msg->twist.linear.x;
    }

    void Safe_Stop::guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg)
    {
        switch (msg->state)
        {
            case cav_msgs::GuidanceState::SAFE_STOP :
                safe_stop_state_= safe_stop_planner::safe_stop_state::ROUTE_END;
                // route_end re;
                // cav_msgs::TrajectoryPlan trajectory = re.plan_trajectory();
                //publish trajectory
                break;
            default:
                safe_stop_state_= safe_stop_planner::safe_stop_state::INACTIVE;
                break;
        }
    }

}