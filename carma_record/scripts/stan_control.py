#!/usr/bin/env python
import rospy
from cav_msgs.msg import RouteState
from autoware_msgs.msg import ControlCommandStamped
from autoware_msgs.msg import Lane
from cav_msgs.msg import TrajectoryPlan
from geometry_msgs.msg import PoseStamped, TwistStamped
from automotive_platform_msgs import SteeringFeedback
from tf.transformations import euler_from_quaternion
import math

class Controller:
    def __init__(self):
        self.current_crosstrack = 0
        self.error_threshold = 0.5 # 1.5m threshold no steering input will be added
        self.waypoints = [] # list of tuple
        self.current_heading = 0.0
        self.target_yaw = 0.0
        self.veh_yaw_rate = 0.0
        self.last_veh_time = None
        self.traj_yaw_rate = 0.0
        self.last_yaw_time = None
        self.prev_steer = 0.0
        self.current_steer = 0.0

        self.K = 0.3 # 0.11 10deg resonse at 1.5m Negative because steering angle is the same direction as crosstrack and we want to reduce crosstrack
        
        self.K_soft = 1.0
        self.K_dyaw = 0.1
        self.K_dsteer = 0.1

        self.ctrl_pub = rospy.Publisher('/guidance/ctrl_raw', ControlCommandStamped, queue_size=1)
        self.ctrl_sub = rospy.Subscriber("/guidance/ctrl_raw_pp", ControlCommandStamped, self.ctrl_callback)
        self.route_state_sub = rospy.Subscriber("/guidance/route_state", RouteState, self.route_callback)
        self.traj_sub = rospy.Subscriber("/guidance/plan_trajectory", TrajectoryPlan, self.traj_callback)
        self.veh_pose_sub = rospy.Subscriber("/localization/current_pose", PoseStamped, self.pose_callback)
        self.vel_sub = rospy.Subscriber("/hardware_interface/vehicle/twist", TwistStamped, self.twist_callback)
        self.vel_sub = rospy.Subscriber("/hardware_interface/steering_feedback", SteeringFeedback, self.steer_callback)

    def route_callback(self, msg):
        self.current_crosstrack = msg.cross_track

    def ctrl_callback(self, msg):
        new_ctrl = msg

        heading_term = self.target_yaw - self.current_heading # omega NOTE: We are ignoring dynamics here
        crosstrack_term = math.atan2(self.K * self.current_crosstrack, self.current_speed + self.K_soft)
        yaw_rate_term = self.K_dyaw * (self.veh_yaw_rate - self.traj_yaw_rate)
        steer_term = self.K_dsteer * (self.prev_steer - self.current_steer)

        print("Terms - Traj Yaw: " + str(self.target_yaw) + " heading: " + str(self.current_heading) + " cross: " + str(self.current_crosstrack) + " speed: " + str(self.current_speed) + " K: " + str(self.K))

        new_ctrl.cmd.steering_angle = heading_term + crosstrack_term + yaw_rate_term + steer_term
        #print("CrossTrack: " + str(self.current_crosstrack) + " P " + str(self.P) + " initial: " + str(msg.cmd.steering_angle))
        print("Final" + str(new_ctrl.cmd.steering_angle))
        self.ctrl_pub.publish(new_ctrl)
    
    def waypoints_callback(self, msg):
        self.waypoints = []
        for wp in msg.waypoints:
            self.waypoints.append((wp.pose.pose.position.x, wp.pose.pose.position.y))

    def traj_callback(self,msg):

        new_yaw = self.target_yaw
        if len(msg.trajectory_points) > 3:

            new_yaw = msg.trajectory_points[2].yaw
        elif len(msg.trajectory_points) > 2:
            new_yaw = msg.trajectory_points[1].yaw

        delta_yaw = new_yaw - self.target_yaw
        
        if self.last_yaw_time != None:
            delta_t = msg.header.stamp - self.last_yaw_time

            self.traj_yaw_rate = delta_yaw / delta_t.to_seconds()
        else:
            self.traj_yaw_rate = 0
        
        self.last_yaw_time = msg.header.stamp
        # Else leave as prev value

    def pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        if self.last_veh_time != None:
            delta_t = msg.header.stamp - self.last_yaw_time
            delta_yaw = yaw - self.current_heading

            self.veh_yaw_rate = delta_yaw / delta_t.to_seconds()
        else:
            self.veh_yaw_rate = 0.0
        self.current_heading = yaw
        self.last_veh_time = msg.header.stamp

    def twist_callback(self, msg):
        self.current_speed = abs(msg.twist.linear.x)
    
    def steer_callback(self, msg):
        self.prev_steer = self.current_steer
        self.current_steer = msg.steering_wheel_angle

    
def lane_control():


    rospy.init_node('lane_control')

    c = Controller()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lane_control()