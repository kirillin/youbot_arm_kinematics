#!/usr/bin/env python
import rospy
import sys

from youbot_arm_control.srv import IK


class IKClient():

    def __init__(self):
        rospy.init_node("ik_test")

    def ikClient(self, xyz, rpy):
        rospy.wait_for_service('ik')
        try:
            ik = rospy.ServiceProxy('ik', IK)
            return ik(xyz, rpy)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def usage(self):
        return "usage:\n %s x y z r p y" % sys.argv[0]


if __name__ == "__main__":
    ikc = IKClient()

    if len(sys.argv) == 7:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        rr = float(sys.argv[4])
        pp = float(sys.argv[5])
        yy = float(sys.argv[6])
    else:
        print(ikc.usage())
        sys.exit(1)

    print("Requesting xyz: (%f; %f; %f)" % (x, y, z))
    print("Requesting rpy: (%f; %f; %f)" % (rr, pp, yy))

    resp = ikc.ikClient((x, y, z), (rr, pp, yy))
    if resp.achievable:
        print("accepted solve!")
    else:
        print("failed solve!")
