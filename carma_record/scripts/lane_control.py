#!/usr/bin/env python
import rospy
from cav_msgs.msg import RouteState
from autoware_msgs.msg import ControlCommandStamped

class Controller:
    def __init__(self):
        self.current_crosstrack = 0
        self.error_threshold = 0.5 # 1.5m threshold no steering input will be added
        self.P = 0.01 # 0.11 10deg resonse at 1.5m Negative because steering angle is the same direction as crosstrack and we want to reduce crosstrack
        self.ctrl_pub = rospy.Publisher('/guidance/ctrl_raw', ControlCommandStamped, queue_size=1)
        self.ctrl_sub = rospy.Subscriber("/guidance/ctrl_raw_pp", ControlCommandStamped, self.ctrl_callback)
        self.route_state_sub = rospy.Subscriber("/guidance/route_state", RouteState, self.route_callback)

    def route_callback(self, msg):
        self.current_crosstrack = msg.cross_track

    def ctrl_callback(self, msg):
        new_ctrl = msg
        print("CrossTrack: " + str(self.current_crosstrack) + " P " + str(self.P) + " initial: " + str(msg.cmd.steering_angle))
        # if abs(self.current_crosstrack) > self.error_threshold:
        #     if self.current_crosstrack >= 0:
        #         new_ctrl.cmd.steering_angle += abs(self.P * self.current_crosstrack)
        #     else:
        #         new_ctrl.cmd.steering_angle -= abs(self.P * self.current_crosstrack)
        if abs(self.current_crosstrack) > self.error_threshold:
            
            new_ctrl.cmd.steering_angle += self.P * self.current_crosstrack
        print("Final" + str(new_ctrl.cmd.steering_angle))
        self.ctrl_pub.publish(new_ctrl)

    
def lane_control():


    rospy.init_node('lane_control')

    c = Controller()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lane_control()