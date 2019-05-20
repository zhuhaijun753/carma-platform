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

// ROS
#include <ros/ros.h>

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
        
        // runs publish at a desired frequency
        int rate;

    private:
        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool readParameters();

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;


};

}  // namespace pure_pursuit_wrapper
