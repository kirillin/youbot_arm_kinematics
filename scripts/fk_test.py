#!/usr/bin/env python
import rospy
import tf

from sensor_msgs.msg import JointState

from libs.kinematics import Kinematics
from config.config import Config


class Checker:

    def __init__(self):
        rospy.init_node("fk_test")

        self.subJointState = rospy.Subscriber("joint_states", JointState, self.callbackJointState)

        self.ks = Kinematics(Config.DH_A, Config.DH_ALPHA,
                             Config.DH_D, Config.DH_THETA)

        rospy.spin()

    def callbackJointState(self, jointState):
        qs = []
        for i, jointPosition in enumerate(jointState.position):
            qs.append(jointPosition)
            # pass griper's fingers frames (and griper base too)
            if i == 4:
                break

        xyz, qtn, rpy, h = self.ks.forward(qs)

        rospy.loginfo(' qs: ' + str(qs))
        rospy.loginfo('xyz: ' + str(xyz) + ' qtn: ' + str(qtn))
        rospy.loginfo('rpy: ' + str(rpy))

        # set orange brick at the end of griper
        br = tf.TransformBroadcaster()
        br.sendTransform(xyz, qtn, rospy.Time.now(), 'test_box', "ground_link")

if __name__ == '__main__':
    Checker()
