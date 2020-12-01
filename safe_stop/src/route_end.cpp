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

#include "route_end.h"
#include <carma_utils/containers/containers.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cav_msgs/TrajectoryPlan.h>
#include <vector>
#include <lanelet2_core/geometry/Point.h>
#include <unordered_set>
#include <safe_stop/smoothing/SplineI.h>
#include <safe_stop/smoothing/CubicSpline.h>
#include <safe_stop/smoothing/filters.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>


namespace safe_stop_planner
{

    cav_msgs::TrajectoryPlan route_end::plan_trajectory()
    {
        ros::WallTime start_time = ros::WallTime::now(); // Start timeing the execution time for planning so it can be logged
        //get position on map
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_downtrack = wm_->routeTrackPos(current_loc).downtrack;
        cav_msgs::ManeuverPlan maneuver_plan;
        
        auto points_and_target_speeds = maneuvers_to_points(maneuver_plan.maneuvers, current_downtrack, wm_);
        auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, 8);
        
        cav_msgs::TrajectoryPlan trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        trajectory.trajectory_points=compose_trajectory_from_centerline(downsampled_points);
        trajectory.initial_longitudinal_velocity= current_speed_;
        
        return trajectory;
    }

    std::vector<PointSpeedPair> route_end::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
        double starting_downtrack, const carma_wm::WorldModelConstPtr& wm)
    {
        std::vector<safe_stop_planner::PointSpeedPair> points_and_target_speeds;
        std::unordered_set <lanelet::Id> visited_lanelets;
        
        //ending distance- Assuming straight line deceleration of 3m/s^2- use as config_param
        double travel_distance = pow(current_speed_,2)/(2*3.0); 

        //traveling from max_starting_downtrack, for a distance of travel_distance
        double ending_downtrack = starting_downtrack + travel_distance;
        //get lanelets between starting and ending
        auto lanelets = wm_->getLaneletsBetween(starting_downtrack,ending_downtrack, true);
        //add lanelets to set
        std::vector<lanelet::ConstLanelet> lanelets_to_add;
        for(auto l : lanelets)
        {
            if(visited_lanelets.find(l.id()) == visited_lanelets.end())
            {
                lanelets_to_add.push_back(l);
                visited_lanelets.insert(l.id());
            }
        }        
        //concatenate lanelets
        lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);
        bool first = true;
        for (auto p : route_geometry)
        {
            if(first && points_and_target_speeds.size() !=0)
            {
                first = false;
                continue;
            }
            PointSpeedPair pair;
            pair.point = p;
            pair.speed = 0.0; //ending speed should be 0
            points_and_target_speeds.push_back(pair);
        }

        return points_and_target_speeds;
        
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> 
    route_end::compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points)
    {
        int nearest_pt_index = getNearestPointIndex(points);

        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end());// Points in front of current vehicle position

        std::vector<DiscreteCurve> sub_curves = compute_sub_curves(future_points); //not constrained to time limit- should be atleast 6s
        
        
        std::vector<double> final_yaw_values;
        std::vector<double> final_actual_speeds;
        std::vector<lanelet::BasicPoint2d> all_sampling_points;

        for(const auto& discrete_curve : sub_curves)
        {
            std::vector<double> speed_limits;
            std::vector<lanelet::BasicPoint2d> curve_points;
            splitPointSpeedPairs(discrete_curve.points, &curve_points, &speed_limits);

            std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points);
            if (!fit_curve)
            {  // TODO how better to handle this case
                for (size_t i = 0; i < discrete_curve.points.size() - 1; i++)
                {
                    Eigen::Isometry2d point_in_map =
                        curvePointInMapTF(discrete_curve.frame, discrete_curve.points[i].point, final_yaw_values.back());
                    all_sampling_points.push_back(point_in_map.translation());
                    final_yaw_values.push_back(final_yaw_values.back());
                    final_actual_speeds.push_back(final_actual_speeds.back());
                }
                continue;
            }
            ROS_DEBUG("Got fit");

            std::vector<lanelet::BasicPoint2d> sampling_points;
            sampling_points.reserve(1+ discrete_curve.points.size() * 2);

            std::vector<double> distributed_speed_limits;
            distributed_speed_limits.reserve(1 + discrete_curve.points.size() * 2);

            //get yaw values
            std::vector<double> yaw_values = carma_wm::geometry::compute_tangent_orientations(sampling_points);
            
            for (int i = 0; i < yaw_values.size() - 1; i++)
            {  // Drop last point
                Eigen::Isometry2d point_in_map = curvePointInMapTF(discrete_curve.frame, sampling_points[i], yaw_values[i]);
                Eigen::Rotation2Dd new_rot(point_in_map.rotation());
                final_yaw_values.push_back(new_rot.smallestAngle());
                all_sampling_points.push_back(point_in_map.translation());
            }
            
        }

        // Add current vehicle point to front of the trajectory
        lanelet::BasicPoint2d cur_veh_point(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        all_sampling_points.insert(all_sampling_points.begin(),cur_veh_point);  // Add current vehicle position to front of sample points

        //get time
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);
        std::vector<double> times;
        trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

        //ensure time bound atleast 6s
        if(times[times.size()-1] - times[0] < 6.0)
        {
            double time_step = times[times.size()-1] - times[times.size()-2];
            while(times[times.size()-1] - times[0] < 6.0)
            {
                all_sampling_points.push_back(all_sampling_points[all_sampling_points.size()-1]);
                times.push_back(times[times.size()-1] + time_step);
            }
        }

        std::vector <cav_msgs::TrajectoryPlanPoint> traj;
        traj.reserve(all_sampling_points.size());

        for(int i=0; i<all_sampling_points.size();i++)
        {
            cav_msgs::TrajectoryPlanPoint tpp;
            ros::Duration relative_time(times[i]);
            tpp.target_time=ros::Time::now() + relative_time;
            tpp.x= all_sampling_points[i].x();
            tpp.y = all_sampling_points[i].y();
            tpp.yaw = final_yaw_values[i];

            tpp.controller_plugin_name = "default";
            tpp.planner_plugin_name = "safe_stop_planner";
            traj.push_back(tpp);
        }
        return traj;
    }
    

    int route_end::getNearestPointIndex(const std::vector<PointSpeedPair>& points)
    {
        lanelet::BasicPoint2d veh_point(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point,veh_point);
            
            if(distance < min_distance)
            {
                best_index = i;
                min_distance = distance;
            }
            i++;
        }
        return best_index;
    }
    
    void route_end::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
    std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    Eigen::Isometry2d route_end::curvePointInMapTF(const Eigen::Isometry2d& curve_in_map, const lanelet::BasicPoint2d& p, double yaw) const
    {
        Eigen::Rotation2Dd yaw_rot(yaw);
        Eigen::Isometry2d point_in_c = carma_wm::geometry::build2dEigenTransform(p, yaw_rot);
        Eigen::Isometry2d point_in_map = curve_in_map * point_in_c;
        return point_in_map;
    }

    std::vector<DiscreteCurve> route_end::compute_sub_curves(const std::vector<PointSpeedPair>& map_points)
    {
        if ( map_points.size() < 2)
        {
            throw std::invalid_argument("Not enough points");
        }

        std::vector<DiscreteCurve> curves;
        DiscreteCurve curve;
        curve.frame = compute_heading_frame(map_points[0].point, map_points[1].point);
        Eigen::Isometry2d map_in_curve = curve.frame.inverse();

        for(size_t i = 0; i< map_points.size() -1;i++)
        {
            lanelet::BasicPoint2d p1 = map_in_curve * map_points[i].point;
            lanelet::BasicPoint2d p2 = map_in_curve * map_points[i + 1].point;  // TODO Optimization to cache this value

            PointSpeedPair initial_pair;
            initial_pair.point = p1;
            initial_pair.speed = map_points[i].speed;
            curve.points.push_back(initial_pair);

            bool x_dir = (p2.x() - p1.x()) > 0;
        
            if (!x_dir)  // If x starts going backwards we need a new curve
            {
                // New Curve
                curves.push_back(curve);

                curve = DiscreteCurve();
                curve.frame = compute_heading_frame(map_points[i].point, map_points[i + 1].point);
                map_in_curve = curve.frame.inverse();

                PointSpeedPair pair;
                pair.point = map_in_curve * map_points[i].point;
                pair.speed = map_points[i].speed;

                curve.points.push_back(pair);  // Include first point in curve
            }
        }
        curves.push_back(curve);
        return curves;
    }

    Eigen::Isometry2d route_end::compute_heading_frame(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2)
    {
        Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

        return carma_wm::geometry::build2dEigenTransform(p1, yaw);
    }

    
    std::unique_ptr<smoothing::SplineI>
    route_end::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
    {
        if (basic_points.size() < 3)
        {
            ROS_WARN_STREAM("Insufficient Spline Points");
            return nullptr;
        }

        std::unique_ptr<smoothing::SplineI> spl = std::make_unique<smoothing::CubicSpline>();
        spl->setPoints(basic_points);

        return spl;
    }
}
