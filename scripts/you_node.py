#!/usr/bin/python

import rospy
import threading
from numpy import arange, zeros, sin, pi, cos

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse

from youbot_arm_kinematics.kinematics import Kinematics
from youbot_arm_kinematics.dh_parameters import YouBotDHParameters


class TestTrajectories:

    def __init__(self):
        self.log_file = open('trajectories_model.txt', 'w')
        self.log_file_xyz = open('trajectories_model_xyz.txt', 'w')

        rospy.init_node("test_trajectories")
        self.lock = threading.Lock()

        self.js_sub = rospy.Subscriber("joint_states", JointState, self.js_callback)
        self.js_pub = rospy.Publisher("joint_states", JointState, queue_size=10)

        self.ks = Kinematics(YouBotDHParameters.DH_A, YouBotDHParameters.DH_ALPHA,
                             YouBotDHParameters.DH_D, YouBotDHParameters.DH_THETA)

        self.loop()
        self.cur_t = 0

    def __del__(self):
        self.log_file.close()
        self.log_file_xyz.close()

    def js_callback(self, js_msg):
        self.lock.acquire()
        cur_angs = js_msg.position
        # cur_vels = js_msg.velocity
        self.lock.release()

        self.log_file.write("{} {} {} {} {} {}\n".format(self.cur_t, cur_angs[0], cur_angs[1], cur_angs[2], cur_angs[3], cur_angs[4]))
                                      # cur_vels[0], cur_vels[1], cur_vels[2], cur_vels[3], cur_vels[4]))

    def getQ(self, t):
        N = 5

        joints_min = [0.025, 0.025, -5.0, 0.025, 0.12]
        joints_max = [5.80, 2.6, -0.025, 3.4, 5.64]

        jointsA = [abs(joints_max[i] - joints_min[i]) for i in range(N)]
        jointsAd2 = [jointsA[i] / 2 for i in range(N)]

        nf = [8, 8, 8, 8, 8]
        T = [9, 6, 7, 4.5, 6]

        a = [  # 1    2      6    4     5      6     7     8
            [-1, -3.00, 0.50, 0.10, 1.00, 0.10, 0.50, 0.10, 3.00],
            [-1, 2.00, 0.10, 1.00, 0.10, 0.00, 0.00, 0.00, 1.50],
            [-1, -3.00, 1.00, 0.50, 1.00, 0.50, 1.00, 1.00, 6.00],
            [-1, -2.00, 0.10, 0.10, 1.00, 0.10, 0.02, 0.10, 1.00],
            [-1, 3.00, 0.10, 1.00, 0.02, 0.10, 0.02, 0.10, 2.00]
        ]

        a = [  # 1    2       3     4     5      6     7     8
            [-1, 3.00, 0.00, 1.00, 1.00, 0.00, 0.00, 1.00, 3.00],
            [-1, 4.00, 1.00, 0.00, 1.10, 0.00, 1.00, 0.00, 0.50],
            [-1, 5.00, 0.00, 0.50, 0.00, 1.50, 0.00, 1.00, 3.00],
            [-1, 6.00, 0.10, -0.20, 0.70, -0.10, 0.50, 0.50, -1.00],
            [-1, 7.00, 0.10, 1.00, 0.02, 0.10, 0.02, 0.10, 2.00]
        ]


        scale = [0.35, 0.29, 0.2, 0.42, 0.45]
        q0 = [jointsAd2[0] - 0.5, jointsAd2[1] - 0.13, -jointsAd2[2] - 0.7, jointsAd2[3] - 0.3, jointsAd2[4] - 0.3]

        q, dq = zeros(N), zeros(N)
        for i in range(N):
            w0 = 2 * pi / T[i]
            for k in range(1, nf[i] + 1):
                q[i] = q[i] + a[i][k] * cos(k * w0 * t)
                dq[i] = dq[i] - a[i][k] * sin(k * w0 * t)
            q[i] = q0[i] + scale[i] * q[i]
            dq[i] = scale[i] * dq[i]
        return q

    def loop(self):

        rate = rospy.Rate(30)
        start_time = rospy.Time.now().to_sec()
        prev_t = 0
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - start_time
            dt = t - prev_t
            prev_t = t
            self.cur_t = t
            # print(t)

            q = self.getQ(t)
            (x, y, z), qtn, rpy, h = self.ks.forward(q)
            self.log_file_xyz.write("{} {} {}\n".format(x, y, z))

            # safe zone: cylinder, plane of floor
            R, z0 = 0.15, 0.4
            x0, y0 = 0, 0
            if (((x - x0)**2 + (y - y0)**2 > R**2) or (z > z0)) and (z > 0.05):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
                            'gripper_finger_joint_l', 'gripper_finger_joint_r']
                msg.position = [0.01] * 7
                msg.position[:5] = q
                self.js_pub.publish(msg)
            else:
                print("COLLISION: " + str(t))
            rate.sleep()


if __name__=="__main__":
    tt = TestTrajectories()
