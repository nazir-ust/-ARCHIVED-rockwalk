#!/usr/bin/env python


"""
Visualizing relevant plots for rock and walk motion:
1. Phase plot + Time
2. Cone roll angle, roll velocity, robot joint angle against time
3. Force topic when you add mass

"""

import sys
import rospy
import copy
import math
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped, AccelStamped, Vector3Stamped, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from tf import transformations as tfms
import tf_conversions as tfc
import numpy as np
import matplotlib
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt
from cone_pose_visualization import cone_manipuland


font = {'family' : 'normal',
        'weight' : 'normal',
        'size'   : 20}

matplotlib.rc('font', **font)

class rock_walk_plot:

    def __init__(self):

        self._init_time = rospy.get_time()

        self.initialize_subscribers()

        self.initialize_data_arrays()


    def initialize_subscribers(self):

        self._contact_position_sub =rospy.Subscriber("contact_position_ginsberg", Vector3, self.store_contact_position_data)
        self._twist_sub =rospy.Subscriber("twist_ginsberg", TwistStamped, self.store_twist_data)
        self._euler_sub =rospy.Subscriber("euler_ginsberg", Vector3, self.store_euler_data)
        # self._kong_eef_position_sub =rospy.Subscriber("kong_eef_position", Point, self.store_kong_eef_position)
        rospy.sleep(2)

    def store_kong_eef_position(self, eef_position):
        self._kong_eef_position = copy.deepcopy(eef_position)

    def store_twist_data(self, twist_data):
        self._twist = copy.deepcopy(twist_data)

    def store_euler_data(self, body_euler_data):
        self._euler = copy.deepcopy(body_euler_data)

    def store_contact_position_data(self, position_data):
        self._contact_position = copy.deepcopy(position_data)

    def current_time_difference(self):

        current_time = rospy.get_time()
        self._current_time_difference = (current_time - self._init_time)

        # print(self._current_time_difference)

    def initialize_data_arrays(self):
        self._time_values = []

        self._euler_ginsberg_values_x = []
        self._euler_ginsberg_values_y = []
        self._euler_ginsberg_values_z = []

        self._twist_ginsberg_values_x = []
        self._twist_ginsberg_values_y = []
        self._twist_ginsberg_values_z = []

        self._ground_contact_position_values_x = []
        self._ground_contact_position_values_y = []


    def append_data_values(self):
        self._time_values.append(self._current_time_difference)

        self._euler_ginsberg_values_x.append(self._euler.x)
        self._euler_ginsberg_values_y.append(self._euler.y)
        self._euler_ginsberg_values_z.append(self._euler.z)

        self._twist_ginsberg_values_x.append(self._twist.twist.angular.x)
        self._twist_ginsberg_values_y.append(self._twist.twist.angular.y)
        self._twist_ginsberg_values_z.append(self._twist.twist.angular.z)

        self._ground_contact_position_values_x.append(self._contact_position.x)
        self._ground_contact_position_values_y.append(self._contact_position.y)


    def setup_contact_position_plot(self):
        fig_contact_plot, axis_contact_plot = plt.subplots(1, 1)
        # axis_contact_plot.set_aspect('equal')
        axis_contact_plot.set_xlim(-20, 20)
        axis_contact_plot.set_ylim(-20, 20)
        axis_contact_plot.hold(True)

        plt.show(False)
        plt.draw()
        plt.xlabel("World x (m)")
        plt.ylabel("World y (m)")
        plt.title("Contact Position Plot")

        background_contact_plot = fig_contact_plot.canvas.copy_from_bbox(axis_contact_plot.bbox)

        points_contact_plot = axis_contact_plot.plot(0, 0, 'b-')[0]

        plt.pause(1e-17)

        return fig_contact_plot, axis_contact_plot, background_contact_plot, points_contact_plot


    def setup_euler_plot(self):
        figure_euler_plot, axis_euler_plot = plt.subplots(1, 1)
        # self._ax.set_aspect('equal')
        axis_euler_plot.set_xlim(0, 90)
        axis_euler_plot.set_ylim(-90,90)

        axis_euler_plot.hold(True)
        plt.show(False)
        plt.draw()
        plt.xlabel("Time (seconds)")
        plt.ylabel("Angles (degrees)")
        plt.title("Euler Plot")

        background_euler_plot = figure_euler_plot.canvas.copy_from_bbox(axis_euler_plot.bbox)


        points_euler_plot_theta = axis_euler_plot.plot(0, 0, 'g-', label='Theta (deg)')[0]
        points_euler_plot_phi = axis_euler_plot.plot(0, 0, 'm-', label='Phi (deg)')[0]

        axis_euler_plot.legend()
        plt.pause(1e-17)

        return figure_euler_plot, axis_euler_plot, background_euler_plot, points_euler_plot_theta, points_euler_plot_phi



    def contact_position_plot(self, figure, axis, background, points):

        points.set_data(self._ground_contact_position_values_x,self._ground_contact_position_values_y)
        figure.canvas.restore_region(background)
        axis.draw_artist(points)
        figure.canvas.blit(axis.bbox)
        plt.pause(1e-50)



    def euler_plot(self, figure, axis, background, points_theta, points_phi):

        points_theta.set_data(self._time_values, self._euler_ginsberg_values_y)
        points_phi.set_data(self._time_values, self._euler_ginsberg_values_z)

        figure.canvas.restore_region(background)
        axis.draw_artist(points_theta)
        axis.draw_artist(points_phi)
        figure.canvas.blit(axis.bbox)

        plt.pause(1e-50)

if __name__ == '__main__':
    rospy.init_node("rock_walk_plots", anonymous=True)


    rw_plot = rock_walk_plot()

    [fig_contact_plot, axis_contact_plot, background_contact_plot, points_contact_plot] = rw_plot.setup_contact_position_plot()

    # [figure_euler_plot, axis_euler_plot, background_euler_plot, points_euler_plot_theta, points_euler_plot_phi] = rw_plot.setup_euler_plot()

    rate = rospy.Rate(50) #previously 55



    while not rospy.is_shutdown():
        rw_plot.current_time_difference()
        rw_plot.append_data_values()

        rw_plot.contact_position_plot(fig_contact_plot, axis_contact_plot, background_contact_plot, points_contact_plot)

        # rw_plot.euler_plot(figure_euler_plot, axis_euler_plot, background_euler_plot, points_euler_plot_theta, points_euler_plot_phi)

        rate.sleep()

    rospy.spin()
