#!/usr/bin/env python

"""
This scripts provides a control algorithm to manipulate an oblique cone object
in a passive dynamic rock and walk manner.

Outline of control algorithm
1. Subscribe to object twist topic. An IMU is affixed to the conic object.
2. When roll angular velocity is zero the end-effector is moved, according to the
waypoints generated by `integral_curves_concatenate.m`. 'Left' or 'Right' motion
of the end-effector is decided based on the (sign with offset) of the roll angle.
"""

import sys
import rospy
import copy
import moveit_commander
import math
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped, AccelStamped, Vector3Stamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from tf import transformations as tfms
import tf_conversions as tfc
import numpy as np
import scipy.io as sio


class manipulation_control_law:


    def __init__(self, group):

        moveit_commander.roscpp_initialize(sys.argv) #Check this line. And also how to control two robots

        self._robot = moveit_commander.RobotCommander("robot_description")
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(group)


        self._group.set_max_velocity_scaling_factor(1)
        self._group.set_max_acceleration_scaling_factor(1)


        self.initialize_publishers()
        self.initialize_tf_broadcasters_listeners()

        self.set_object_parameters()

    def set_object_parameters(self):
        self._object_mass = 1
        self._object_radius = 0.35
        self._object_masscenter_height = 0.30
        self._object_masscenter_lateraloffset = 0.15

    def intialize_arm_pose(self):

        self._kong_arm_init_pose = Pose()

        self._kong_arm_init_pose.position.x =  -1.65#-1.65
        self._kong_arm_init_pose.position.y = -0.53#-0.47
        self._kong_arm_init_pose.position.z = 0.55#0.49

        self._kong_arm_init_pose.orientation.x = -0.0260040764497
        self._kong_arm_init_pose.orientation.y = -0.701264425535
        self._kong_arm_init_pose.orientation.z = 0.712262067535
        self._kong_arm_init_pose.orientation.w = 0.0153212479218

        self.follow_target_pose(self._kong_arm_init_pose)

        # self._kong_arm_init_pose = copy.deepcopy(self._group.get_current_pose().pose)

    def get_current_pose(self):
        return self._group.get_current_pose()


    def follow_target_joint(self, joint_target):
        rospy.loginfo("Following Joint Target")

        try:
            self._group.go(joint_target, wait=True)

        except rospy.ROSInterruptException:
            pass


    def follow_target_pose(self, data):

        rospy.loginfo("Following Target Pose with End Effectors Now")

        try:
            self._group.set_pose_target(data, "kong_ee_link")

            plan = self._group.plan()

            """
            The command self._group.execute() will execute real robot arm motion.
            Be careful before you uncomment this command and take all the necessary precautions.
            Keep the emergency stop button close to stop the motion of the robot
            Uncomment this command after carefully visualizing the motion in RViz.
            """

            self._group.execute(plan)

            self._group.clear_pose_targets()

        except rospy.ROSInterruptException:
            pass

    def initialize_publishers(self):
        rospy.loginfo("Initializing Publishers")
        self._hong_active_joint_angle_pub = rospy.Publisher('hong_active_joint_angles', Vector3, queue_size=10)
        self._kong_eef_position_pub = rospy.Publisher('kong_eef_position', Point, queue_size=10)


    def initialize_tf_broadcasters_listeners(self):
        self._broadcaster = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()

    def subscribe_object_twist(self):
        rospy.loginfo("Subscribing to Twist Topic")
        self._twist_sub =rospy.Subscriber("twist_ginsberg", TwistStamped, self.store_twist_data)

    def store_twist_data(self, twist_data):
        self._twist = copy.deepcopy(twist_data)

    def subscribe_body_euler(self):
        rospy.loginfo("Subscribing to Body Euler Topic")
        self._body_euler_sub =rospy.Subscriber("euler_ginsberg", Vector3, self.store_body_euler)

    def store_body_euler(self, body_euler_data):
        self._body_euler = copy.deepcopy(body_euler_data)


    def relocate_robot_arm_feedforward_control(self, rock_sign):

        if rock_sign == 1:
            print("right")
            self.relocate_arm_right_jointspace()
            rock_sign = -1*rock_sign
            rospy.sleep(0.23) #feedforward parameter. adjust this.

        else:
            print("left")
            self.relocate_arm_left_jointspace()
            rock_sign = -1*rock_sign
            rospy.sleep(0.23) #feedforward parameter

        return rock_sign


    def relocate_robot_arm_feedback(self, rock_sign):
        """in this control scheme, take action when zero velocity event is detected"""

        if self._twist.twist.angular.y < 23 and self._twist.twist.angular.y > 0 and self._body_euler.y < 0  and rock_sign == 1:
            # self.relocate_arm_right(count_right_rock) #angular velocity is positive first

            print("right")
            pitch_error = self._body_euler.x - (-100)
            self.relocate_arm_right_jointspace(pitch_error)
            rock_sign = -1*rock_sign

        if self._twist.twist.angular.y > -23 and self._twist.twist.angular.y < 0 and self._body_euler.y  > 0 and rock_sign == -1:
            #try elif above
            # self.relocate_arm_left(count_left_rock) #angular velocity is negative first

            print("left")
            pitch_error = self._body_euler.x - (-100)
            self.relocate_arm_left_jointspace(pitch_error)
            rock_sign = -1*rock_sign

        return rock_sign


    def relocate_robot_arm_feedback_IK(self, rock_sign, count_right_rock, count_left_rock):
        """in this control scheme, take action when zero velocity event is detected"""

        if self._twist.twist.angular.z > -23 and self._twist.twist.angular.z < -3 and self._body_euler.z < 0  and rock_sign == 1:

            print("right")
            self.relocate_arm_right(count_right_rock)
            rock_sign = -1*rock_sign
            count_right_rock += 1

        if self._twist.twist.angular.z < 23 and self._twist.twist.angular.z > 3 and self._body_euler.z  > 0 and rock_sign == -1:
            #try elif above

            print("left")
            self.relocate_arm_left(count_left_rock)
            rock_sign = -1*rock_sign
            count_left_rock += 1

        return rock_sign, count_right_rock, count_left_rock




    def relocate_robot_arm_threshold_control(self, rock_sign):

        if abs(self._twist.twist.angular.y) > 30:

            if self._body_euler.y < 0  and rock_sign == 1:
                print("right")
                self.relocate_arm_right_jointspace()
                rock_sign = -1*rock_sign

            elif self._body_euler.y  > 0 and rock_sign == -1:

                # self.relocate_arm_left(count_left_rock) #angular velocity is negative first

                print("left")
                self.relocate_arm_left_jointspace()
                rock_sign = -1*rock_sign

        return rock_sign


    def relocate_robot_arm_threshold_control_IK(self, rock_sign, count_right_rock, count_left_rock):

        if abs(self._twist.twist.angular.y) > 30:

            if self._body_euler.y < 0  and rock_sign == 1:
                print("right")
                self.relocate_arm_right(count_right_rock)
                rock_sign = -1*rock_sign
                count_right_rock += 1

            elif self._body_euler.y  > 0 and rock_sign == -1:

                # self.relocate_arm_left(count_left_rock) #angular velocity is negative first

                print("left")
                self.relocate_arm_left(count_left_rock)
                rock_sign = -1*rock_sign
                count_left_rock += 1

        return rock_sign, count_right_rock, count_left_rock


    def relocate_arm_right(self, count_right_rock):
        """
        NOT IN USE
        First do a straight line. Later changes thi to corresponding flow
        Start point: current location of effector."""

        end_pose = Pose()
        #note the signs below. maltab frame didnot align with world frame here. (-y,x)
        end_pose.position.x = self._kong_arm_init_pose.position.x - self._right_rocking_target[count_right_rock, 1]
        end_pose.position.y = self._kong_arm_init_pose.position.y + self._right_rocking_target[count_right_rock, 0]
        end_pose.position.z = self._kong_arm_init_pose.position.z
        end_pose.orientation = copy.deepcopy(self._kong_arm_init_pose.orientation)
        self.follow_target_pose(end_pose)

    def relocate_arm_left(self, count_left_rock):
        """
        NOT IN USE
        First do a straight line"""

        end_pose = Pose()
        #note the signs below. maltab frame didnot align with world frame here. (-y,x)
        end_pose.position.x = self._kong_arm_init_pose.position.x - self._left_rocking_target[count_left_rock, 1]
        end_pose.position.y = self._kong_arm_init_pose.position.y + self._left_rocking_target[count_left_rock, 0]
        end_pose.position.z = self._kong_arm_init_pose.position.z
        end_pose.orientation = copy.deepcopy(self._kong_arm_init_pose.orientation)
        self.follow_target_pose(end_pose)


    def relocate_arm_right_jointspace(self, pitch_error):

        joint_target = self._group.get_current_joint_values()

        joint_target[0] -= math.radians(3.65)
        joint_target[1] -= math.radians(1)
        joint_target[2] += math.radians(1.8)
        joint_target[3] -= math.radians(.62) #wrist1

        self.follow_target_joint(joint_target)


    def relocate_arm_left_jointspace(self, pitch_error):

        joint_target = self._group.get_current_joint_values()

        joint_target[0] += math.radians(3.25)
        joint_target[1] -= math.radians(1)
        joint_target[2] += math.radians(1.8)
        joint_target[3] -= math.radians(.62) #wrist1

        self.follow_target_joint(joint_target)



    def publish_end_effector_pose(self):
        eef_position = copy.deepcopy(self._group.get_current_pose().pose.position)
        self._kong_eef_position_pub.publish(eef_position)



    def publish_hong_active_joint_values(self):
        joint_values = self._group.get_current_joint_values()

        msg = Vector3()

        msg.x = math.degrees(joint_values[0])
        msg.y = math.degrees(joint_values[1])
        msg.z = math.degrees(joint_values[2])

        self._hong_active_joint_angle_pub.publish(msg)


    def import_left_right_rocking_matlab(self):
        # lr_rocking = sio.loadmat('/home/nazir/Documents/MATLAB/CDRM_Modeling/lr_rocking.mat')
        lr_rocking = sio.loadmat('/home/nazir/ws_moveit/src/ur10_cm/matlab_scripts/lr_rocking.mat')

        self._right_rocking_target = lr_rocking['right_rock']
        self._left_rocking_target = lr_rocking['left_rock']






if __name__ == '__main__':
    rospy.init_node("kong_rigid_effector", anonymous=True)

    manipulator_control = manipulation_control_law("kong_arm")

    manipulator_control.subscribe_object_twist()
    manipulator_control.subscribe_body_euler()


    rospy.sleep(1)

    print(manipulator_control.get_current_pose())
    manipulator_control.intialize_arm_pose()
    rospy.sleep(2)



    manipulator_control.import_left_right_rocking_matlab()

    nb_key = raw_input('Press 1 to begin manipulation')

    try:
        number_key = int(nb_key)
    except ValueError:
        print("Invalid number")

    if number_key == 1:

        rate = rospy.Rate(50) #Rate previously 200. Change to smaller value
        rock_sign = -1; #-1 for left rock. 1 for right rock. To make sure only alternate rocks
        count_right_rock = 0
        count_left_rock = 0
        rocking_steps = 15
        while not rospy.is_shutdown():


            # rock_sign = manipulator_control.relocate_robot_arm_feedback(rock_sign)

            # rock_sign = manipulator_control.relocate_robot_arm_threshold_control(rock_sign)
            # rock_sign = manipulator_control.relocate_robot_arm_feedforward_control(rock_sign)

            if count_left_rock == 0:
                init_time = rospy.get_time()
                # print(init_time)


            rock_sign, count_right_rock, count_left_rock = manipulator_control.relocate_robot_arm_feedback_IK(rock_sign,
                                                                                                        count_right_rock,
                                                                                                        count_left_rock)

            if count_left_rock == rocking_steps:
                elapsed_time = rospy.get_time() - init_time
                print(elapsed_time)

            # rock_sign, count_right_rock, count_left_rock = manipulator_control.relocate_robot_arm_threshold_control_IK(rock_sign,
            #                                                                                                     count_right_rock,
            #                                                                                                     count_left_rock)

            rate.sleep()

    rospy.spin()
