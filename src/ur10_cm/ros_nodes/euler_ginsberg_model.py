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
from std_msgs.msg import Header, ColorRGBA
from tf import transformations as tfms
import tf_conversions as tfc
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt




class EulerTransform:
    def __init__(self):

        self.initialize_publishers()
        self.initialize_subscribers()


    def initialize_publishers(self):
        self._ginsberg_euler_pub = rospy.Publisher('euler_ginsberg', Vector3, queue_size=10)
        self._ginsberg_twist_pub = rospy.Publisher('twist_ginsberg', Vector3, queue_size=10)
        self._marker_pub = rospy.Publisher('manipuland_marker_topic', Marker, queue_size=10)


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


    def transform_to_ginsberg_euler_twist(self):

        #convert quaterion to rotation matrix

        ######################################################
        "Some additions here"
        earth_initIMU_quat = [0.32720142,0.6336514,-0.61558544,-0.33537993]
        earth_world_quat = tfms.quaternion_multiply(earth_initIMU_quat, tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz'))

        # print(earth_world_quat)

        # earth_world_quat = [ 0.0078735, -0.01568597,  0.67480203,  0.73779006]

        #[-0.02563422,  0.02972348,  0.72386144,  0.68882801] -old one

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
        twist_ginsberg = np.matmul(np.transpose(rot_nonhomo),twist_vec)#rot.dot(twist_vec)

        self._ginsberg_twist = Vector3()
        self._ginsberg_twist.x = twist_ginsberg[0]
        self._ginsberg_twist.y = twist_ginsberg[1]
        self._ginsberg_twist.z = twist_ginsberg[2]

        self._ginsberg_twist_pub.publish(self._ginsberg_twist)

        # print(self._ginsberg_twist)




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

    rate = rospy.Rate(50)


    while not rospy.is_shutdown():

        euler_transform.transform_to_ginsberg_euler_twist()
        euler_transform.publish_manipuland_marker()
        rate.sleep()

    rospy.spin()
