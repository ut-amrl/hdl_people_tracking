#!/usr/bin/env python

import rospy
import roslib
NODE_NAME = 'ToHumanState'
roslib.load_manifest(NODE_NAME)
from amrl_msgs.msg import HumanStateMsg
from amrl_msgs.msg import HumanStateArrayMsg
from hdl_people_tracking.msg import TrackArray

human_pub = rospy.Publisher("human_states", HumanStateArrayMsg)

def DetectionCallback(msg):
    stateArray = HumanStateArrayMsg()

    # Create a human for each track
    for track in msg.tracks:
        human = HumanStateMsg()
        human.id = track.id
        human.pose.x = track.pos.x
        human.pose.y = track.pos.y
        human.pose.theta = 0.0
        human.translational_velocity.x = track.vel.x
        human.translational_velocity.y = track.vel.y
        human.rotational_velocity = 0.0
    stateArray.human_states.append(human)
    human_pub.publish(stateArray)

def main():
    rospy.init_node('ToHumanState')
    rospy.Subscriber('/hdl_people_tracking_nodelet/tracks', TrackArray, DetectionCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
