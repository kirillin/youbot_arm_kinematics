#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np
from math import cos, sin, pi

m = 0.7


def f(l, angle, x0, y0):
    """returns end point for a link"""
    # d**2 = (x - x0)**2 + (y - y0)**2
    oy = l * sin(angle) + y0
    ox = l * cos(angle) + x0
    return ox, oy


def draw1dEx(a, d, q, iq=(2, 3, 4), point=(0, 0, 0), point_gripper=(0, 0, 0)):
    """ only for KUKAyoubot"""
    x = np.zeros(6)
    y = np.zeros(6)
    x[0], y[0] = 0, 0
    x[1], y[1] = 0, d[0]
    x[2], y[2] = a[1], d[1]+d[0]
    sum_angles = pi/2
    sum_angles += q[1]
    x[3], y[3] = f(a[2], sum_angles, x[2], y[2])
    sum_angles += q[2]
    x[4], y[4] = f(a[3], sum_angles, x[3], y[3])
    sum_angles += q[3] - pi/2
    x[5], y[5] = f(d[5], sum_angles, x[4], y[4])
    return x, y


num = 1

def plotIK(ks, q, h):
    # number of solve (has been selected by radio buttons)
    # initialization of the plot
    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.50, bottom=0.50)
    # min/max values for axis
    plt.xlim(-m, m)
    plt.ylim(-m, m)
    plt.hlines(0, -m, m)
    plt.vlines(0, -m, m)
    plt.grid(True)

    # init solve for start
    x, y, z = h[:3, 3]
    xx, yy = draw1dEx(ks.get_dh_a(), ks.get_dh_d(), q[num], iq=(2, 3, 5),
                      point=(x, y, z), point_gripper=(x, y, z))

    # plot diff usefullests
    plt.plot((0, 0), (0, -10), 'g', (0, 0), (0, -10), 'bo', linewidth=3.0)
    # solve for given point
    l0, l1, = plt.plot(xx, yy, 'g', xx, yy, 'bo', linewidth=3.0)
    # given point in space
    ll, = plt.plot(x, z, 'mo')
    # draw red circles as workspace, where q1 in [-pi; pi]
    cir1 = plt.Circle((xx[2], yy[2]), 0.290 + 0.218, color='r')
    cir2 = plt.Circle((xx[2] - 0.066, yy[2]), 0.290 + 0.218, color='r')
    ax.add_artist(cir1)
    ax.add_artist(cir2)

    # trackbars
    axcolor = 'lightgoldenrodyellow'
    axq1 = plt.axes([0.25, 0.1, 0.65, 0.03])
    axq2 = plt.axes([0.25, 0.15, 0.65, 0.03])
    axq3 = plt.axes([0.25, 0.20, 0.65, 0.03])
    axq4 = plt.axes([0.25, 0.25, 0.65, 0.03])
    axq5 = plt.axes([0.25, 0.30, 0.65, 0.03])

    sq1 = Slider(axq1, 'q1', -pi, pi, valinit=0)
    sq2 = Slider(axq2, 'q2', -pi, pi, valinit=0)
    sq3 = Slider(axq3, 'q3', -pi, pi, valinit=0)
    sq4 = Slider(axq4, 'q4', -pi, pi, valinit=0)
    sq5 = Slider(axq5, 'q5', -pi, pi, valinit=0)

    # updater drawing
    def update(val):
        # set angles
        q1 = sq1.val
        q2 = sq2.val
        q3 = sq3.val
        q4 = sq4.val
        q5 = sq5.val
        # solve FK for setting angles
        x, q, r, h = ks.forward([0, 0, q2, q3, q4, 0, 0])
        print x, r
        # solve IK for solved FK for setting angles
        qs = ks.inverse(x, r, h)
        # draw configuration of the manipulator
        # the magenta point is given!
        x, y, z = h[:3, 3]
        xx, yy = draw1dEx(ks.get_dh_a(), ks.get_dh_d(), qs[num], iq=(2, 3, 5),
                          point=(x, y, z), point_gripper=(x, y, z))
        # update data
        l0.set_xdata(xx)
        l0.set_ydata(yy)
        l1.set_xdata(xx)
        l1.set_ydata(yy)
        ll.set_xdata(x)
        ll.set_ydata(z)
        fig.canvas.draw_idle()

    sq1.on_changed(update)
    sq2.on_changed(update)
    sq3.on_changed(update)
    sq4.on_changed(update)
    sq5.on_changed(update)

    # button stuff
    resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
    button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')

    def reset(event):
        sq1.reset()
        sq2.reset()
        sq3.reset()
        sq4.reset()
        sq5.reset()

    button.on_clicked(reset)

    # rax = plt.axes([0.025, 0.5, 0.15, 0.15])
    # radio = RadioButtons(rax, (0, 1), active=0)
    #
    # def changeSolve(value):
    #     num = int(value)
    #     q = qs[num]
    #     sq1.set_val(q[0])
    #     sq2.set_val(q[1])
    #     sq3.set_val(q[2])
    #     sq4.set_val(q[3])
    #     sq5.set_val(q[4])
    #
    # radio.on_clicked(changeSolve)

    # commit
    plt.show()
