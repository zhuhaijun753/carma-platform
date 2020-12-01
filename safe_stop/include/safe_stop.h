#pragma once
/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/GuidanceState.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/Geometry.h>
#include <geometry_msgs/TwistStamped.h>

namespace safe_stop_planner
{
    enum safe_stop_state{   //For use with multiple safe_stop maneuvers
        INACTIVE=0,
        ROUTE_END=1,
    };

    /**
    * \brief Convenience class for pairing 2d points with speeds
    */ 
    struct PointSpeedPair
    {
        lanelet::BasicPoint2d point;
        double speed = 0;
    };

    /**
     * \brief Class representing a curve in space defined by a set of discrete points in the specified frame
     */ 
    struct DiscreteCurve
    {
        Eigen::Isometry2d frame; // Frame which points are in
        std::vector<PointSpeedPair> points;
    };

    /** 
     * Primary work class for the safe stop planner package
     * Governs the behavior when a safe stop is required at the
     * end of a route. More use cases to be added in the future.
    */

    class Safe_Stop
    {
        public:
        /**
         * \brief Default constructor for Safe_Stop class
         */
        //Safe_Stop();

        void run();
        
        //State of the safe stop planner, inactive by default
        safe_stop_state safe_stop_state_;

        protected:
        //wm listener pointer and pointer to the actual wm object
        std::shared_ptr <carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;

        //Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;
        
        // CARMA ROS node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

        //ROS publishers and subscribers
        ros::Subscriber guidance_state_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;
        ros::Publisher safe_stop_trajectory_pub_;

        //Current speed of vehicle when safe stop is called
        double current_speed_ = 0.0;
        
        private:
        /**
         * \brief Initialize ROS publishers, subscribers, service servers and service clients
         */
        void initialize();
        
        /**
         * \brief Callback for the pose subscriber, which will store latest pose locally
         * \param msg Latest pose message
         */
        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

        /**
         * \brief Callback for the twist subscriber, which will store latest twist locally
         * \param msg Latest twist message
         */
        void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);
        
        void guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg);

    };
        
}