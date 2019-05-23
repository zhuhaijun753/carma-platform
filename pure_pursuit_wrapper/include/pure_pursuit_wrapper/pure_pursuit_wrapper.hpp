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

#include <mutex>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/TrajectoryPlan.h>

// autoware
#include "autoware_msgs/Lane.h"
#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"

namespace pure_pursuit_wrapper {

/*!
 * Main class for the node to handle the ROS interfacing.
 */

class PurePursuitWrapper {
    public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        PurePursuitWrapper(ros::NodeHandle& nodeHandle);

        /*!
        * Destructor.
        */
        virtual ~PurePursuitWrapper();

        // @brief ROS initialize.
        void Initialize();
        
        // runs publish at a desired frequency
        int rate;

        // Shutdown flags and mutex
        std::mutex shutdown_mutex_;
        bool shutting_down_ = false;


    private:

        //@brief ROS node handle.
        ros::NodeHandle& nh_;

        //@brief ROS subscribers.
        ros::Subscriber system_alert_sub_;
        ros::Subscriber trajectory_plan_sub_;

        // @brief ROS publishers.
        ros::Publisher way_points_pub_;
        // ros::Publisher current_pose_pub_;
        ros::Publisher current_velocity_pub_;
        ros::Publisher system_alert_pub_;

        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool ReadParameters();

        // @brief ROS subscriber handlers.
        void SystemAlertHandler(const cav_msgs::SystemAlert::ConstPtr& msg);
        void TrajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& msg);

        // @brief ROS pusblishers.

        // void PublisherForConfig(const autoware_config_msgs::ConfigWaypointFollowerConstPtr &config);
        // void PublisherForCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void PublisherForCurrentVelocity(geometry_msgs::TwistStamped& msg);
        void PublisherForWayPoints(autoware_msgs::Lane& msg);


        /*
         * @brief Handles caught exceptions which have reached the top level of this node
         * 
         * @param message The exception to handle
         * 
         * If an exception reaches the top level of this node it should be passed to this function.
         * The function will try to log the exception and publish a FATAL message to system_alert before shutting itself down.
         */
        void HandleException(const std::exception& e);

        // @brief Shutsdown this node
        void Shutdown();

};

}  // namespace pure_pursuit_wrapper
