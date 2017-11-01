#!/usr/bin/env python
import rospy
from math import cos, sin, atan2, pi, sqrt
from scipy import cross, dot, transpose
import tf.transformations as tt
import numpy as np

from ploter import plotIK


class Kinematics:

    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

    def set_dh_parameters(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

    def get_dh_d(self):
        return self.d

    def get_dh_a(self):
        return self.a

    def get_dh_theta(self):
        return self.theta


    def forward(self, qs, start=0, end=5):
        """ FK problem """
        h = tt.identity_matrix()
        for i in range(start, end):
            rz = tt.rotation_matrix(self.theta[i] - qs[i], (0, 0, 1))
            tz = tt.translation_matrix((0, 0, self.d[i]))
            tx = tt.translation_matrix((self.a[i], 0, 0))
            rx = tt.rotation_matrix(self.alpha[i], (1, 0, 0))
            a = tt.concatenate_matrices(rz, tz, tx, rx)
            h = tt.concatenate_matrices(h, a)
            out = "%f\t%f\t%f\t%f + %f" % (self.a[i], self.alpha[i],
                                           self.d[i], qs[i], self.theta[i])
            rospy.logdebug(out)
        xyz = h[:3, 3]
        qtn = tt.quaternion_from_matrix(h)
        rpy = tt.euler_from_matrix(h[:3, :3], axes='sxyz')
        return xyz, qtn, rpy, h


    def getProjectionH(self, t):
        """
            a method of projection a given general goal orientation
                into the subspace of KUKAYoubot' arm
        """
        # TODO don't work :(
        # divide T(4x4) matrix of R(3x3) and vector p(3x1)
        px, py, pz = t[:3, 3]
        r = t[:3, :3]
        # divide R(3x3) matrix on components
        xt, yt, zt = r[:3, 0], r[:3, 1], r[:3, 2]
        # normal vector to plane of the manipulator
        pxy = sqrt(px ** 2 + py ** 2)
        #m = dot(1 / pxy, [-py, px, 0])
        m = [0,-1,0]
        # normal vector to both zt and m
        k = cross(zt, m)
        # new vector zt
        pzt = cross(m, k)
        # t is angle between zt and plane of manipulator
        cost = dot(zt, pzt)
        sint = dot(cross(pzt, zt), k)
        # Rodrigues' formula
        pyt = dot(cost, yt) + dot(sint, cross(yt, k)) + \
            dot(dot((1 - cost), dot(k, yt)), k)
        pxt = cross(pyt, pzt)
        # new rotate matrix
        pr = transpose([pxt, pyt, pzt])
        t[:3, :3] = pr
        return t


    def getVector1to4Frame(self, theta1, pr, p):
        """ the vector for solving geometric part of IK """
        px, py, pz = p
        c1, s1 = cos(theta1), sin(theta1)
        xr = (py - self.d[4]*pr[1,2]) * s1 + (px - self.d[4]*pr[0,2]) * c1 - self.a[0]
        yr = pz - self.d[0] - self.d[4] * pr[2, 2]
        zr = 0.0
        return xr, yr, zr


    def inverse(self, xyz, rpy, t=None):
        """ IK problem"""
        eps = np.finfo(np.float).eps
        qs = np.zeros(20).reshape(4, 5)
        # success indicator for 2 sets of angles "q"
        sols = [True, True]

        # The transformation matrix t is 4x4
        # need for projection method
        t = tt.euler_matrix(rpy[0], rpy[1], rpy[2])
        t[:3, 3] = xyz
        # TODO don't work
        # t = self.getProjectionH(t)

        px, py, pz = t[:3, 3]
        pr = t[:3, :3]

        # theta_1
        theta_1_I = atan2(py, px)
        theta_1_II = atan2(-py, -px)

        # theta_5
        s5 = pr[0, 0] * sin(theta_1_I) - pr[1, 0] * cos(theta_1_I)
        c5 = pr[0, 1] * sin(theta_1_I) - pr[1, 1] * cos(theta_1_I)
        theta_5_I = atan2(s5, c5)
        s5 = pr[0, 0] * sin(theta_1_II) - pr[1, 0] * cos(theta_1_II)
        c5 = pr[0, 1] * sin(theta_1_II) - pr[1, 1] * cos(theta_1_II)
        theta_5_II = atan2(s5, c5)

        # theta_234
        c234 = -pr[2, 2]
        s234 = pr[0, 2] * cos(theta_1_I) + pr[1, 2] * sin(theta_1_I)
        theta_234_I = atan2(s234, c234)
        s234 = pr[0, 2] * cos(theta_1_II) + pr[1, 2] * sin(theta_1_II)
        theta_234_II = atan2(s234, c234)

        # theta_3
        # cosine theorem
        xr, yr, zr = self.getVector1to4Frame(theta_1_I, pr, [px, py, pz])
        xr -= eps     # for singulars -pi for q2
        numerator_0 = xr**2 + yr**2 + zr**2 - self.a[1]**2 - self.a[2]**2
        denominator_0 = 2 * self.a[1] * self.a[2]
        # for singulars \pm pi/2 and \pm pi plus 100 * eps with sign of numerator
        cosq3_I = numerator_0 / denominator_0 - np.sign(numerator_0) * 100 * eps

        if cosq3_I <= 1:
            theta_3_I = -atan2(sqrt(1.0 - cosq3_I ** 2), cosq3_I)
            theta_3_II = atan2(sqrt(1.0 - cosq3_I ** 2), cosq3_I)

            phi_I = atan2(yr, xr)
            beta_I = atan2(self.a[2] * sin(abs(theta_3_I)), self.a[1] + self.a[2] * cos(abs(theta_3_I)))
            beta_II = atan2(self.a[2] * sin(abs(theta_3_II)), self.a[1] + self.a[2] * cos(abs(theta_3_II)))

            theta_2_I = phi_I + beta_I
            theta_2_II = phi_I - beta_II

            theta_4_I  = theta_234_I - theta_2_I - theta_3_I
            theta_4_II = theta_234_I - theta_2_II - theta_3_II

            qs[0] = [theta_1_I, theta_2_I, theta_3_I, theta_4_I, theta_5_I]
            qs[1] = [theta_1_I, theta_2_II, theta_3_II, theta_4_II, theta_5_I]
        else:
            print("NO SOLUTIONS for q_I and q_II")
            print("cosq3: %.20f" % cosq3_I)
            sols[0] = False

        xr, yr, zr = self.getVector1to4Frame(theta_1_II, pr, [px, py, pz])
        xr -= eps     # for singulars -pi for q2
        numerator_0 = xr**2 + yr**2 + zr**2 - self.a[1]**2 - self.a[2]**2
        denominator_0 = 2 * self.a[1] * self.a[2]
        # for singulars \pm pi/2 and \pm pi plus 100 * eps with sign of numerator
        cosq3_II = numerator_0 / denominator_0 - np.sign(numerator_0) * 100 * eps

        if cosq3_II <= 1:
            theta_3_III = -atan2(sqrt(1.0 - cosq3_II ** 2), cosq3_II)
            theta_3_IV = atan2(sqrt(1.0 - cosq3_II ** 2), cosq3_II)

            phi_II = atan2(yr, xr)
            beta_III = atan2(self.a[2] * sin(abs(theta_3_III)), self.a[1] + self.a[2] * cos(abs(theta_3_III)))
            beta_IV = atan2(self.a[2] * sin(abs(theta_3_IV)), self.a[1] + self.a[2] * cos(abs(theta_3_IV)))

            theta_2_III = phi_II + beta_III
            theta_2_IV = phi_II - beta_IV

            theta_4_III  = theta_234_II - theta_2_III - theta_3_III
            theta_4_IV = theta_234_II - theta_2_IV - theta_3_IV

            qs[2] = [theta_1_II, theta_2_III, theta_3_III, theta_4_III, theta_5_II]
            qs[3] = [theta_1_II, theta_2_IV, theta_3_IV, theta_4_IV, theta_5_II]
        else:
            print("NO SOLUTIONS for q_III and q_IV")
            print("cosq3: %.20f" % cosq3_II)
            sols[1] = False

        for i in range(4):
            for j in range(5):
                qs[i][j] = self.theta[j] - qs[i][j]
                qs[i][j] = self.normalizedAngle(qs[i][j], j)

        return qs, sols


    def normalizedAngle(self, q, j):
        if j != 2:
            while q < 0.0:
                q += 2*pi
            while q >= 2*pi:
                q -= 2*pi
        else:
            while q > 0.0:
                q -= 2*pi
            while q <= -2*pi:
                q += 2*pi
        return q


if __name__ == '__main__':
    DH_A = (0.033, 0.155, 0.135, 0, 0)
    DH_ALPHA = (pi/2, 0, 0, pi/2, 0)
    DH_D = (0.147, 0, 0, 0, 0.218)
    DH_THETA = (PI*169.0/180.0, PI*65.0/180.0+PI/2, -PI*146.0/180.0, PI*102.5/180.0+PI/2, PI*167.5/180.0)  #theta = DH_THETA - q

    # check ik
    ks = Kinematics(DH_A, DH_ALPHA, DH_D, DH_THETA)

    x, q, r, h = ks.forward([0.0, 0.0, 0.0, 0.0, 0.0])

    q = ks.inverse(x, r, h)
    print(q)

    plotIK(ks, q, h)
