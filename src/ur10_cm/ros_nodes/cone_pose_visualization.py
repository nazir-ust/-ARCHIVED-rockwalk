#!/usr/bin/env python


"""
This node serves the following purpose:
visualize object pose as Marker
publish relative orientation of subscribe_object_twist

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
# from hong_rigid_effector_control import manipulation_control_law



class cone_manipuland:
    def __init__(self):

        self.initialize_publishers()
        self.initialize_tf_broadcasters_listeners()
        self.initialize_subscribers()
        self.initialize_attributes()

    def initialize_attributes(self):
        self._zero_velocity_time = []
        self._zero_velociy_tilt = []

    def initialize_publishers(self):
        self._marker_pub = rospy.Publisher('manipuland_marker_topic', Marker, queue_size=10)
        self._marker_effector_pub = rospy.Publisher('effector_marker_topic', Marker, queue_size=10)
        self._body_euler_pub = rospy.Publisher('body_euler', Vector3, queue_size=10)
        self._object_postion_world_pub = rospy.Publisher('object_pose_world', Pose, queue_size=10)


    def initialize_tf_broadcasters_listeners(self):
        self._broadcaster = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()


    def initialize_subscribers(self):
        self.subscribe_object_position()
        self.subscribe_object_quaternion()
        self.subscribe_object_twist()
        rospy.sleep(1)


    def subscribe_object_position(self):
        # Comment this line if you are using april tag.
        self._object_pose_from_tag = Pose(Point(0,0,0),Quaternion(0,0,0,1))

        rospy.loginfo("Subscribing to Object Position from April Tag")
        self._april_tag_sub = rospy.Subscriber("tag_detections_pose", PoseArray, self.store_object_april_tag_pose)

    def subscribe_object_twist(self):
        rospy.loginfo("Subscribing to Twist Topic")
        self._twist_sub =rospy.Subscriber("twist_motion_shield", TwistStamped, self.store_twist_data)

    def subscribe_object_quaternion(self):
        rospy.loginfo("Subscribing to Quaternion Topic")
        self._quaternion_sub =rospy.Subscriber("quat_motion_shield", QuaternionStamped, self.store_quaternion_data)

    def subscriber_object_euler(self):
        rospy.loginfo("Subscribing to Euler Topic")
        self._euler_sub =rospy.Subscriber("euler_motion_shield", Vector3Stamped, self.store_euler_data)

    def store_object_april_tag_pose(self, pose_array):
        # if len(pose_array.poses) == 2:
        #     self._object_pose_from_tag = copy.deepcopy(pose_array.poses[0])
        #     self._object_pose_from_tag.position.x = 0.5*(pose_array.poses[0].position.x + pose_array.poses[1].position.x)
        #     self._object_pose_from_tag.position.y = 0.5*(pose_array.poses[0].position.y + pose_array.poses[1].position.y)
        #     self._object_pose_from_tag.position.z = 0.5*(pose_array.poses[0].position.z + pose_array.poses[1].position.z)
        #     return



        if len(pose_array.poses) > 0:

            if pose_array.poses[0]:
                self._object_pose_from_tag = copy.deepcopy(pose_array.poses[0])
                # print(self._object_pose_from_tag)
                return
            # if pose_array.poses[1]:
            #     self._object_pose_from_tag = copy.deepcopy(pose_array.poses[1])
            #     return



    def store_twist_data(self, twist_data):
        self._twist = copy.deepcopy(twist_data)

    def store_quaternion_data(self, quaternion_data):
        self._quaternion = copy.deepcopy(quaternion_data)

        self._unit_imu_quaternion = tfms.unit_vector([self._quaternion.quaternion.x/1000,
                                              self._quaternion.quaternion.y/1000,
                                              self._quaternion.quaternion.z/1000,
                                              self._quaternion.quaternion.w/1000])

        # print(self._unit_imu_quaternion)

    def store_euler_data(self, euler_data):
        self._euler = copy.deepcopy(euler_data)


    def broadcast_transforms(self):

        # try:
        #     (tran,rot) = self._listener.lookupTransform('/world', '/hong_tool0', rospy.Time(0))
        #     self._broadcaster.sendTransform((tran[0],tran[1],tran[2]),
        #                     (0, 0, 0, 1),
        #                     rospy.Time.now(),
        #                     "anchor", #child frame
        #                     "world") #parent frame. transfrom
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass
        #
        #
        # self._broadcaster.sendTransform((self._anchor_imu_position[0],self._anchor_imu_position[1],self._anchor_imu_position[2]),
        #                 (self._anchor_imu_vec_quat[0], self._anchor_imu_vec_quat[1], self._anchor_imu_vec_quat[2], self._anchor_imu_vec_quat[3]),
        #                 rospy.Time.now(),
        #                 "imu",
        #                 "anchor")



        self._broadcaster.sendTransform((self._world_imu_pose.position.x,self._world_imu_pose.position.y,self._world_imu_pose.position.z),
                        (self._world_imu_pose.orientation.x, self._world_imu_pose.orientation.y, self._world_imu_pose.orientation.z, self._world_imu_pose.orientation.w),
                        rospy.Time.now(),
                        "imu",
                        "world")

        self._broadcaster.sendTransform((self._world_camera_position[0],self._world_camera_position[1],self._world_camera_position[2]),
                        (self._world_camera_quat[0], self._world_camera_quat[1], self._world_camera_quat[2], self._world_camera_quat[3]),
                        rospy.Time.now(),
                        "usb_cam",
                        "world")

        self._broadcaster.sendTransform((self._world_cone_pose.position.x,self._world_cone_pose.position.y,self._world_cone_pose.position.z),
                        (self._world_cone_pose.orientation.x, self._world_cone_pose.orientation.y, self._world_cone_pose.orientation.z, self._world_cone_pose.orientation.w),
                        rospy.Time.now(),
                        "cone_marker",
                        "world")


    def publish_body_euler(self):

        # imu_initIMU_euler = np.degrees(tfms.euler_from_quaternion(self._imu_initIMU))

        world_imu_euler = np.degrees(tfms.euler_from_quaternion([self._world_imu_pose.orientation.x,
                                                                 self._world_imu_pose.orientation.y,
                                                                 self._world_imu_pose.orientation.z,
                                                                 self._world_imu_pose.orientation.w]))

        self._body_euler = Vector3()
        self._body_euler.x = world_imu_euler[0]
        self._body_euler.y = world_imu_euler[1]
        self._body_euler.z = world_imu_euler[2]
        self._body_euler_pub.publish(self._body_euler)


    def compute_relative_imu_orientation(self):
        """name convention: parentFrame_childFrame_quat"""


        cone_imu_vec_quat = tfms.quaternion_from_euler(0, 0, math.pi, 'rxyz')


        earth_initImu_vec_quat = [-0.71086415,  0.02551225,  0.00909408,  0.70280765]


        initImu_anchor_vec_quat = tfms.quaternion_from_euler(math.pi/2, 0, math.pi, 'rxyz')

        earth_anchor_vec_quat = tfms.quaternion_multiply(earth_initImu_vec_quat, initImu_anchor_vec_quat)

        self._anchor_imu_vec_quat = tfms.quaternion_multiply(tfms.quaternion_inverse(earth_anchor_vec_quat), self._unit_imu_quaternion)

        anchor_cone = tfms.quaternion_multiply(self._anchor_imu_vec_quat,tfms.quaternion_inverse(cone_imu_vec_quat))


        self._relative_body_quaternion = Quaternion(anchor_cone[0], anchor_cone[1], anchor_cone[2], anchor_cone[3])

    def compute_relative_body_imu_position(self):

        #The following computes the transform of IMU frame relative to its initial frame
        anchor_initIMU = tfms.quaternion_from_euler(-math.pi/2, 0, 0, 'rxyz')
        self._imu_initIMU = tfms.quaternion_multiply(self._anchor_imu_vec_quat, tfms.quaternion_inverse(anchor_initIMU))

        anchor_body_initPosition = [0.350, -0.70, -1.40 , 0]
        anchor_body_position = np.matmul(tfms.quaternion_matrix(self._imu_initIMU), anchor_body_initPosition)

        anchor_imu_initPosition = [0, 0, anchor_body_initPosition[2]+0.50, 0]
        self._anchor_imu_position = np.matmul(tfms.quaternion_matrix(self._imu_initIMU), anchor_imu_initPosition)

        self._anchor_body_position_point = Point(anchor_body_position[0], anchor_body_position[1], anchor_body_position[2])


    def compute_imu_pose(self):

        """This function should compute pose of IMU in world frame"""

        self._world_camera_position = [-0.943419533992, -0.509536543674, 0.111264370752]



        # self._world_camera_quat = tfms.quaternion_from_euler(-math.pi/2,- math.pi/2, 0, 'rxyz')

        world_camera_init = tfms.quaternion_from_euler(-math.pi/2,- math.pi/2, 0, 'rxyz')

        self._world_camera_quat = tfms.quaternion_multiply(world_camera_init, tfms.quaternion_from_euler(math.radians(-28.5),0,0,'rxyz'))

        camera_imu_position_cameraframe = [self._object_pose_from_tag.position.x,
                                           self._object_pose_from_tag.position.y,
                                           self._object_pose_from_tag.position.z,
                                           0]

        camera_imu_position = np.matmul(tfms.quaternion_matrix(self._world_camera_quat), camera_imu_position_cameraframe)

        world_imu_position = [self._world_camera_position[0] + camera_imu_position[0],
                              self._world_camera_position[1] + camera_imu_position[1],
                              self._world_camera_position[2] + camera_imu_position[2]]



        "Some additions here"
        earth_initIMU_quat = [-0.71889202, -0.00903345, -0.02179016,  0.69472142]
        earth_world_quat = tfms.quaternion_multiply(earth_initIMU_quat, tfms.quaternion_from_euler(math.pi/2,0, math.pi/2, 'rxyz'))

        # print(earth_world_quat)

        # earth_world_quat = [ 0.0078735, -0.01568597,  0.67480203,  0.73779006]

        #[-0.02563422,  0.02972348,  0.72386144,  0.68882801] -old one



        world_imu_quat = tfms.quaternion_multiply(tfms.quaternion_inverse(earth_world_quat), self._unit_imu_quaternion)

        self._world_imu_pose = Pose(Point(world_imu_position[0], world_imu_position[1], world_imu_position[2]),
                                Quaternion(world_imu_quat[0], world_imu_quat[1], world_imu_quat[2], world_imu_quat[3]))


        self.compute_object_pose(world_imu_position, world_imu_quat)




    def compute_object_pose(self, world_imu_position, world_imu_quat):

        imu_cone_position_imuframe = [0.35, 0.465, -0.70, 0]
        imu_cone_position = np.matmul(tfms.quaternion_matrix(world_imu_quat),imu_cone_position_imuframe) #converts to world coordinates

        world_cone_position = [world_imu_position[0] + imu_cone_position[0],
                               world_imu_position[1] + imu_cone_position[1],
                               world_imu_position[2] + imu_cone_position[2]]

        cone_imu_quat = tfms.quaternion_from_euler(0, 0, math.pi, 'rxyz')
        world_cone_quat = tfms.quaternion_multiply(world_imu_quat, cone_imu_quat)


        self._world_cone_pose = Pose(Point(world_cone_position[0], world_cone_position[1], world_cone_position[2]),
                                Quaternion(world_cone_quat[0], world_cone_quat[1],world_cone_quat[2], world_cone_quat[3]))



        # print (self._world_cone_pose)

    def publish_object_position(self):
        self._object_postion_world_pub.publish(self._world_cone_pose)


    def publish_manipuland_marker(self):

        marker = Marker(
                    type=Marker.MESH_RESOURCE,
                    action=Marker.ADD,
                    id=0,
                    # lifetime= rospy.Duration(),
                    # pose=Pose(self._anchor_body_position_point, self._relative_body_quaternion),
                    pose = self._world_cone_pose,
                    scale=Vector3(0.001, .001, .001),
                    header=Header(frame_id="world"),
                    color=ColorRGBA(.70,.70,.70 ,1),
                    mesh_resource="file:///home/nazir/ws_moveit/src/ur10_cm/models/right_cone.dae"
                    )
        self._marker_pub.publish(marker)


    def publish_effector_marker(self):

        eelink_eff_quat = tfms.quaternion_from_euler(math.pi,0, math.pi/2, 'rxyz')

        marker_effector = Marker(
                    type=Marker.MESH_RESOURCE,
                    action=Marker.ADD,
                    id=0,
                    # lifetime= rospy.Duration(),
                    # pose=Pose(self._anchor_body_position_point, self._relative_body_quaternion),
                    pose = Pose(Point(0,0.05,0.12), Quaternion(eelink_eff_quat[0],eelink_eff_quat[1],eelink_eff_quat[2],eelink_eff_quat[3])),
                    scale=Vector3(0.001, .001, .001),
                    header=Header(frame_id="kong_ee_link"),
                    color=ColorRGBA(.9,.9,.9 ,1),
                    mesh_resource="file:///home/nazir/ws_moveit/src/ur10_cm/models/Eff_Torus_DAE.dae"
                    )
        self._marker_effector_pub.publish(marker_effector)




    def record_time_oscillation(self):


        if abs(self._twist.twist.angular.y) < 4 and self._body_euler.y < -5:

            self._zero_velocity_time.append(rospy.get_time())
            self._zero_velociy_tilt.append(-self._body_euler.x)
            print("timed")
            if len(self._zero_velocity_time) > 1:
                self.compute_period_oscillation()
            rospy.sleep(0.20)


    def compute_period_oscillation(self):

        diff = np.diff(self._zero_velocity_time)

        self._period_oscillation = np.average(diff)


        self._average_tilt = np.average(self._zero_velociy_tilt)

        print(str(self._period_oscillation) + "\t" + str(self._average_tilt) )








if __name__ == '__main__':
    rospy.init_node("cone_manipuland", anonymous=True)

    cone = cone_manipuland()
    # kong_manipulator_control = manipulation_control_law("kong_arm")

    rate = rospy.Rate(50)


    # cycle = 1; #first time when velocity is zero
    while not rospy.is_shutdown():
        #cone.compute_relative_imu_orientation()
        #cone.compute_relative_body_imu_position()

        cone.compute_imu_pose()
        cone.broadcast_transforms()
        cone.publish_manipuland_marker()
        cone.publish_effector_marker()
        cone.publish_body_euler()
        cone.publish_object_position()
        # kong_manipulator_control.publish_end_effector_pose()

        rate.sleep()

    rospy.spin()
