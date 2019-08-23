#!/usr/bin/env python

"""
17 July 2019

Convert quaternion from IMU to Euler angles according to Ginsberg model.
"""

import sys
import rospy
import copy
import math
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped, AccelStamped, Vector3Stamped
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, Float64
from tf import transformations as tfms
import tf_conversions as tfc
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy import integrate

import matlab.engine



class EulerTransform:
    def __init__(self):

        rospy.sleep(3)

        self.set_object_parameters(object_id=0)
        self.initialize_past_velocity_containers()

        self.initialize_publishers()
        self.initialize_subscribers()


    #     #----------------------------------------------------------------------
    #     self._engine = matlab.engine.start_matlab()
    #
    #     [self._ke_fun, self.pe_fun] = compute_ke_pe_fun(self._object_radius,
    #                                                     self._object_masscenter_lateraloffset,
    #                                                     self._object_masscenter_height,
    #                                                     self._object_mass)
    #
    #
    # def compute_ke_pe(self):




    def set_object_parameters(self, object_id):

        if object_id == 0:
            """Old Metal Cone"""
            self._object_mass = 1

            self._object_radius = 0.35

            self._object_apex_lateral_offset = 0.35
            self._object_apex_vertical_offset = 1.30

            self._object_masscenter_lateraloffset = 0.15
            self._object_masscenter_height = 0.30

        elif object_id == 1:
            self._object_radius = 0.1420


    def initialize_past_velocity_containers(self):
        self._past_velocity_x = np.zeros(1)
        self._past_velocity_y = np.zeros(1)
        self._past_velocity_z = np.zeros(1)

        self._past_velocity_time = []

    def initialize_publishers(self):
        self._ginsberg_euler_pub = rospy.Publisher('euler_ginsberg', Vector3, queue_size=10)
        self._ginsberg_twist_pub = rospy.Publisher('twist_ginsberg', TwistStamped, queue_size=10)
        self._ginsberg_position_pub = rospy.Publisher('position_ginsberg', Vector3, queue_size=10)
        self._ginsberg_contact_position_pub = rospy.Publisher('contact_position_ginsberg', Vector3, queue_size=10)
        self._rocking_amplitude_pub = rospy.Publisher('rocking_amplitude', Float64, queue_size=10)

        # self._marker_pub = rospy.Publisher('manipuland_marker_topic', Marker, queue_size=10)


    def initialize_subscribers(self):
        self.subscribe_object_quaternion()
        self.subscribe_object_twist()
        rospy.sleep(1)


    def subscribe_object_quaternion(self):
        rospy.loginfo("Subscribing to Quaternion Topic")
        self._quaternion_sub =rospy.Subscriber("quat_motion_shield", QuaternionStamped, self.store_quaternion_data)


    def subscribe_object_twist(self):
        rospy.loginfo("Subscribing to Twist Topic")
        self._twist_sub =rospy.Subscriber("twist_motion_shield", TwistStamped, self.store_twist_data)

    def store_quaternion_data(self, quaternion_data):
        self._quaternion = copy.deepcopy(quaternion_data)

        self._unit_imu_quaternion = tfms.unit_vector([self._quaternion.quaternion.x/1000,
                                                      self._quaternion.quaternion.y/1000,
                                                      self._quaternion.quaternion.z/1000,
                                                      self._quaternion.quaternion.w/1000])

        # print(self._unit_imu_quaternion)


    def store_twist_data(self, twist_data):
        self._twist = copy.deepcopy(twist_data)


    def observe_initIMU_quat(self):
        rospy.loginfo("Bring the object to its initial configuration:")
        rospy.sleep(1)
        self._earth_initIMU_quat = self._unit_imu_quaternion
        rospy.loginfo("Initial IMU quaternion recorded")

    def transform_to_ginsberg_euler_twist(self):

        """
        Assume that your world frame aligns with the world frame in Ginsberg.
        """
        "Some additions here"
        # earth_initIMU_quat = [0.32720142,0.6336514,-0.61558544,-0.33537993]

        # earth_initIMU_quat = [-0.71404478,-0.00671382,0.0,0.70006784]

        # earth_world_quat = tfms.quaternion_multiply(self._earth_initIMU_quat, tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz'))

        earth_world_quat = tfms.quaternion_multiply(self._earth_initIMU_quat, tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz'))

        imu_ginsberg_quat = tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz')
        earth_ginsberg_quat = tfms.quaternion_multiply(self._unit_imu_quaternion,imu_ginsberg_quat)
        world_ginsberg_quat = tfms.quaternion_multiply(tfms.quaternion_inverse(earth_world_quat), earth_ginsberg_quat)


        # world_imu_quat = tfms.quaternion_multiply(tfms.quaternion_inverse(earth_world_quat), self._unit_imu_quaternion)


        ######################################################


        rot = tfms.quaternion_matrix(world_ginsberg_quat)

        if rot[2,2]!= 1 or rot[2,2]!= 1:
            theta = math.atan2(math.sqrt(math.pow(rot[0,2],2) + math.pow(rot[1,2],2)),rot[2,2])
            phi = math.atan2(rot[2,1],-rot[2,0])
            psi = math.atan2(-rot[0,2],rot[1,2])

        elif rot[2,2]== -1:
            theta = math.pi/2
            phi = 0
            psi = math.atan2(rot[0,0], -rot[1,0])

        elif rot[2,2]== 1:
            theta = 0
            phi = 0
            psi = math.atan2(-rot[0,0],rot[1,0])

        self._ginsberg_euler = Vector3()
        self._ginsberg_euler.x = np.degrees(psi)
        self._ginsberg_euler.y = np.degrees(theta)
        self._ginsberg_euler.z = np.degrees(phi)

        self._ginsberg_euler_pub.publish(self._ginsberg_euler)

        self._ginsberg_quaternion = tfms.quaternion_from_matrix(rot)





        #########################TWIST TRANSFORM###########################


        twist_vec = np.array([[self._twist.twist.angular.x],[self._twist.twist.angular.y],[self._twist.twist.angular.z]])
        rot_nonhomo = tfms.quaternion_matrix(imu_ginsberg_quat)
        rot_nonhomo = rot_nonhomo[0:3,0:3]
        angular_velocity_ginsberg = np.matmul(np.transpose(rot_nonhomo),twist_vec)#rot.dot(twist_vec)

        self._angular_velocity_ginsberg = Vector3()
        self._angular_velocity_ginsberg.x = angular_velocity_ginsberg[0]
        self._angular_velocity_ginsberg.y = angular_velocity_ginsberg[1]
        self._angular_velocity_ginsberg.z = angular_velocity_ginsberg[2]

        # self._ginsberg_angular_velocity_pub.publish(self._angular_velocity_ginsberg)


        ########################################################################
        #-- Translational Velocity from Nonholonomy of Contact

        trans_vel_x = self._object_radius*math.cos(psi)*math.cos(theta)*np.radians(self._angular_velocity_ginsberg.x) - \
                      self._object_radius*math.sin(psi)*math.sin(theta)*np.radians(self._angular_velocity_ginsberg.y) + \
                      self._object_radius*math.cos(psi)*np.radians(self._angular_velocity_ginsberg.z)

        trans_vel_y = self._object_radius*math.sin(psi)*math.cos(theta)*np.radians(self._angular_velocity_ginsberg.x) - \
                      self._object_radius*math.cos(psi)*math.sin(theta)*np.radians(self._angular_velocity_ginsberg.y) + \
                      self._object_radius*math.sin(psi)*np.radians(self._angular_velocity_ginsberg.z)

        trans_vel_z = self._object_radius*math.cos(theta)*np.radians(self._angular_velocity_ginsberg.y)

        self._translational_velocity_nonholonomic = Vector3()
        self._translational_velocity_nonholonomic.x = trans_vel_x
        self._translational_velocity_nonholonomic.y = trans_vel_y
        self._translational_velocity_nonholonomic.z = trans_vel_z


        self._object_twist = TwistStamped()

        self._object_twist.header.stamp = rospy.Time.now()

        self._object_twist.twist.angular = copy.deepcopy(self._angular_velocity_ginsberg)
        self._object_twist.twist.linear = copy.deepcopy(self._translational_velocity_nonholonomic)

        self._ginsberg_twist_pub.publish(self._object_twist)


    def nonholonomic_translational_distance(self, current_position, current_time):

        new_time = self._object_twist.header.stamp.to_sec()

        position_change_x = self._object_twist.twist.linear.x * (new_time - current_time)
        position_change_y = self._object_twist.twist.linear.y * (new_time - current_time)
        # position_change_z = self._object_twist.twist.linear.z * (new_time - current_time)

        disk_center_position = Vector3()

        disk_center_position.x = current_position.x + position_change_x
        disk_center_position.y = current_position.y + position_change_y
        # new_position.z = current_position.z + position_change_z
        disk_center_position.z = self._object_radius*math.sin(np.radians(self._ginsberg_euler.y))

        self._ginsberg_position_pub.publish(disk_center_position)

        # plt.scatter(disk_center_position.x, disk_center_position.y)

        self.compute_contact_coordinates(disk_center_position)

        return disk_center_position, new_time


    def compute_contact_coordinates(self, disk_center_position):



        rot_psi = tfms.rotation_matrix(np.radians(self._ginsberg_euler.x), [0,0,1])
        init_rot = tfms.rotation_matrix(math.pi/2, [0,0,1])
        rot_theta = tfms.rotation_matrix(np.radians(self._ginsberg_euler.y), [0,1,0])
        rot_phi = tfms.rotation_matrix(np.radians(self._ginsberg_euler.z), [0,0,1])

        rot_StoQ = np.matmul(np.matmul(rot_psi, init_rot),rot_theta)
        rot_StoB = np.matmul(rot_StoQ, rot_phi)

        ground_contact_vector = np.matmul(rot_StoQ,np.array([[self._object_radius],[0],[0],[0]]))

        contact_position_x = disk_center_position.x + ground_contact_vector[0]
        contact_position_y = disk_center_position.y + ground_contact_vector[1]

        contact_position = Vector3()
        contact_position.x = contact_position_x
        contact_position.y = contact_position_y
        contact_position.z = 0

        self._ginsberg_contact_position_pub.publish(contact_position)

        # plt.scatter(contact_position_x, contact_position_y)



    def compute_rocking_amplitude(self, current_rocking_amplitude):
        rocking_amplitude = Float64()
        if np.absolute(self._angular_velocity_ginsberg.z) < 8 and self._ginsberg_euler.y > 10:
            current_rocking_amplitude = np.absolute(self._ginsberg_euler.z)
            self._rocking_amplitude_pub.publish(current_rocking_amplitude)
            return current_rocking_amplitude

        else:
            self._rocking_amplitude_pub.publish(current_rocking_amplitude)
            return current_rocking_amplitude





    def publish_manipuland_marker(self):

        np_quat = self._ginsberg_quaternion

        cone_pose = Pose(Point(0,0,0), Quaternion(np_quat[0],np_quat[1],np_quat[2],np_quat[3]))
        marker = Marker(
                    type=Marker.MESH_RESOURCE,
                    action=Marker.ADD,
                    id=0,
                    # lifetime= rospy.Duration(),
                    # pose=Pose(self._anchor_body_position_point, self._relative_body_quaternion),
                    pose = cone_pose,
                    scale=Vector3(0.001, .001, .001),
                    header=Header(frame_id="world"),
                    color=ColorRGBA(.70,.70,.70 ,1),
                    mesh_resource="file:///home/nazir/ws_moveit/src/ur10_cm/models/right_cone.dae"
                    )

        self._marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node("euler_ginsberg_transform", anonymous=True)

    euler_transform = EulerTransform()

    euler_transform.observe_initIMU_quat()

    rate = rospy.Rate(50)

    # init_time = rospy.get_time()

    current_position = Vector3(0,0,0)
    current_time = rospy.get_time()

    current_rocking_amplitude = Float64(0)

    while not rospy.is_shutdown():

        euler_transform.transform_to_ginsberg_euler_twist()
        [current_position, current_time] = euler_transform.nonholonomic_translational_distance(current_position, current_time)
        # euler_transform.publish_manipuland_marker()
        # euler_transform.compute_contact_coordinates()
        current_rocking_amplitude = euler_transform.compute_rocking_amplitude(current_rocking_amplitude)
        rate.sleep()

    # plt.show()

    rospy.spin()
