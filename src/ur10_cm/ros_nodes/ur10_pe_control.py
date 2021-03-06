#!/usr/bin/env python


import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import math
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped, AccelStamped, Vector3Stamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, Float64
from tf import transformations as tfms
import tf_conversions as tfc
import numpy as np
import scipy.io as sio
from scipy.interpolate import griddata
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import LinearNDInterpolator
from scipy import signal
import scipy
import matplotlib.pyplot as plt


class manipulation_control_law:


    def __init__(self, group):

        moveit_commander.roscpp_initialize(sys.argv) #Check this line. And also how to control two robots

        self._robot = moveit_commander.RobotCommander("robot_description")
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(group)


        print(self._robot.get_group_names())
        print(self._group.get_end_effector_link())

        self._group.set_max_velocity_scaling_factor(1)
        self._group.set_max_acceleration_scaling_factor(1)

        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_tf_broadcasters_listeners()

        self.set_object_parameters(object_id = 1)
        self.load_control_vectorfield()


        print(self._group.get_current_pose().pose)

        rospy.sleep(1)

    #add table to scene:

    def set_object_parameters(self, object_id):

        if object_id == 0:
            rospy.loginfo("Big Cone Object not for Treadmill")
            self._object_mass = 1

            self._object_radius = 0.35

            self._object_apex_lateral_offset = 0.42
            self._object_apex_vertical_offset = 1.30

            self._object_masscenter_lateraloffset = 0.15
            self._object_masscenter_height = 0.30

            self._apex_length = 1.30#1.30

        elif object_id == 1:
            rospy.loginfo("Small Cone Object for Treadmill")
            self._object_mass = 1

            self._object_radius = 0.155

            self._object_apex_lateral_offset = 0.110
            self._object_apex_vertical_offset = 1.10

            self._object_masscenter_lateraloffset = 0.10
            self._object_masscenter_height = 0.50


    def intialize_arm_pose(self):

        self._kong_arm_init_pose = Pose()

        self._kong_arm_init_pose.position.x = -0.80
        self._kong_arm_init_pose.position.y = 0.30
        self._kong_arm_init_pose.position.z = 0.45

        self._kong_arm_init_pose.orientation.x = -0.0260040764497
        self._kong_arm_init_pose.orientation.y = -0.701264425535
        self._kong_arm_init_pose.orientation.z = 0.712262067535
        self._kong_arm_init_pose.orientation.w = 0.0153212479218



        self.follow_target_pose(self._kong_arm_init_pose)

#         self._kong_arm_init_pose = copy.deepcopy(self._group.get_current_pose().pose)


    def follow_target_pose(self, data):

        rospy.loginfo("Following Target Pose with End Effector Now")

        try:
            self._group.set_pose_target(data, "ee_link")
            plan = self._group.plan()
            self._group.execute(plan, wait=True)
            self._group.clear_pose_targets()
        except rospy.ROSInterruptException:
            pass

    def initialize_subscribers(self):
        rospy.loginfo("Initializing Subscribers")
        self._euler_sub =rospy.Subscriber("euler_ginsberg", Vector3, self.store_euler_data)
        self._twist_sub =rospy.Subscriber("twist_ginsberg", TwistStamped, self.store_twist_data)


    def initialize_publishers(self):
        rospy.loginfo("Initializing Publishers")
        self._hong_active_joint_angle_pub = rospy.Publisher('hong_active_joint_angles', Vector3, queue_size=10)
        self._kong_eef_position_pub = rospy.Publisher('kong_eef_position', Point, queue_size=10)


        self._object_pe_pub = rospy.Publisher('object_potential_energy', Float64, queue_size=10)
        self._euler_error_pub = rospy.Publisher('euler_ginsberg_euler', Vector3, queue_size=10)


    def initialize_tf_broadcasters_listeners(self):
        self._broadcaster = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()

    def store_twist_data(self, twist_data):
        self._twist = copy.deepcopy(twist_data) #in degrees per second

    def store_euler_data(self, euler_data):
        self._euler = copy.deepcopy(euler_data) #in degrees


    def rock_walk(self, dir_rock, rock_number):

        #Robot should take the first step instead of manually disturbing the object with hand:

        theta_desired = math.radians(20)

        phi_desired = math.radians(15)

        if rock_number == 0:

            #--------------------------------------------------
            # rospy.loginfo("Recording Initial Heading")
            # self._initial_heading = self._euler.x
            # print("Initial heading is: ", self._initial_heading)
            #--------------------------------------------------

            rospy.loginfo("Initial rocking step to the right")
            rospy.sleep(1)

            self.compute_control_vector_field(theta_desired, phi_desired)

            start_pt = np.zeros((2,1))
            start_pt[0,0] = math.radians(self._euler.y) #tilt angle
            start_pt[1,0] = 0.0 #alpha; start from zero

            self.flow_of_vector_field(start_pt)
            self.compute_apex_position_sequence(dir_rock)
            self.relocate_arm_waypoints()

            dir_rock = -1*dir_rock
            rock_number += 1

            return dir_rock, rock_number


        # if self._euler.z<10 and self._twist.twist.angular.z < -3 and self._euler.y > 10 and dir_rock == -1:

        if self._euler.y > 10 and dir_rock == -1:

            rospy.loginfo("Right Action")


            self.compute_control_vector_field(theta_desired, phi_desired)


            start_pt = np.zeros((2,1))
            start_pt[0,0] = math.radians(self._euler.y) #tilt angle
            start_pt[1,0] = 0.0 #alpha; start from zero

            self.flow_of_vector_field(start_pt)
            self.compute_apex_position_sequence(dir_rock)
            self.relocate_arm_waypoints()

            dir_rock = -1*dir_rock
            rock_number += 1

            # plt.plot(self._stream_theta0.T, self._stream_alpha.T)
            # plt.hold



        # elif self._euler.z > -10 and self._twist.twist.angular.z > 3 and self._euler.y > 10 and dir_rock == 1:

        elif self._euler.y > 10 and dir_rock == 1:


            rospy.loginfo("Left Action")

            self.compute_control_vector_field(theta_desired, phi_desired)


            start_pt = np.zeros((2,1))
            start_pt[0,0] = math.radians(self._euler.y) #tilt angle
            start_pt[1,0] = 0.0 #alpha; start from zero

            self.flow_of_vector_field(start_pt)
            self.compute_apex_position_sequence(dir_rock)
            self.relocate_arm_waypoints()

            dir_rock = -1*dir_rock
            rock_number += 1

            # plt.plot(self._stream_theta0.T, self._stream_alpha.T)
            # plt.hold

        return dir_rock, rock_number

    def relocate_arm_waypoints(self):

        waypoints = []

        current_pose = copy.deepcopy(self._group.get_current_pose().pose)
        # waypoints.append(current_pose)

        for col in self._diff_apex_position_sequence.T:
            wpose = Pose()
            wpose.orientation = copy.deepcopy(current_pose.orientation)
            wpose.position.x = current_pose.position.x + col[0]
            wpose.position.y = current_pose.position.y + col[1]
            wpose.position.z = current_pose.position.z + col[2]

            waypoints.append(wpose)


        (plan, fraction) = self._group.compute_cartesian_path(waypoints,0.01,0.0)


        rospy.loginfo("Following Target Waypoints")

        self._group.execute(plan, wait=True)

        rospy.sleep(1.3)
        self._group.stop()
        self._group.clear_pose_targets()


    def load_control_vectorfield(self):

        e_control = sio.loadmat('/home/nazir/Documents/MATLAB/PDOLRWM/cone_falling_juns_approach/energy_control.mat')

        x_field = e_control['THETA0']
        y_field = e_control['ALPHA']
        # u_field = e_control['U']
        # v_field = e_control['V']

        self._theta_fun = e_control['THETA_FUN']
        self._phi_fun = e_control['PHI_FUN']


        self._x_field_flat = x_field.flatten().reshape(len(x_field.flatten()),1)
        self._y_field_flat = y_field.flatten().reshape(len(y_field.flatten()),1)
        # self._u_field_flat = u_field.flatten().reshape(len(u_field.flatten()),1)
        # self._v_field_flat = v_field.flatten().reshape(len(v_field.flatten()),1)

        self._data_points = np.concatenate((self._x_field_flat, self._y_field_flat), axis = 1)


    def compute_control_vector_field(self, theta_desired, phi_desired):

        """
        Both theta_desired and phi_desired must be in radians.
        """

        obj_fun = np.sqrt(np.square(self._theta_fun-theta_desired) + np.square(self._phi_fun-phi_desired))

        [v_alpha,u_theta0] = np.gradient(obj_fun)

        field_norms = np.sqrt(np.square(u_theta0) + np.square(v_alpha))

        u_theta0_unit = np.divide(-u_theta0,field_norms)
        v_alpha_unit = np.divide(-v_alpha,field_norms)

        self._u_field_flat = u_theta0_unit.flatten().reshape(len(u_theta0_unit.flatten()),1)
        self._v_field_flat = v_alpha_unit.flatten().reshape(len(v_alpha_unit.flatten()),1)


    def flow_of_vector_field(self, start_pt):


        steps = 3
        step_length = 0.02

        stream_x = np.zeros((1, int(math.ceil(steps/step_length))))
        stream_y = np.zeros((1, int(math.ceil(steps/step_length))))


        stream_x[0,0] = start_pt[0,0]
        stream_y[0,0] = start_pt[1,0]


        current_stream_pt = start_pt

        current_direction = np.zeros((2,1))

        linear_interp_u = LinearNDInterpolator(self._data_points, self._u_field_flat)
        linear_interp_v = LinearNDInterpolator(self._data_points, self._v_field_flat)

        current_direction[0,0] = linear_interp_u(start_pt[0,0], start_pt[1,0])
        current_direction[1,0] = linear_interp_v(start_pt[0,0], start_pt[1,0])


        next_direction = np.zeros((2,1))

        for stp in range(1,int(math.ceil(steps/step_length))):


            next_stream_pt = current_stream_pt + step_length*current_direction

            next_direction[0,0] = linear_interp_u(next_stream_pt[0,0], next_stream_pt[1,0])
            next_direction[1,0] = linear_interp_v(next_stream_pt[0,0], next_stream_pt[1,0])

            stream_x[0, stp] = next_stream_pt[0,0];
            stream_y[0, stp] = next_stream_pt[1,0];

            current_stream_pt = next_stream_pt;
            current_direction = next_direction;

            if np.linalg.norm(current_direction) < 0.5:
                break

        self._stream_theta0 = stream_x[0, 0:stp];
        self._stream_alpha = stream_y[0, 0:stp];



    def euler_from_matrix(self,rot):

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

        return psi, theta, phi

    def compute_apex_position(self, rot_StoQ, rot_StoB, G_pos):

        #add rho for general oblique cone. vec_OApex need to be changed.

        R = self._object_radius
        r = self._object_apex_lateral_offset
        l = self._object_apex_vertical_offset

        d = self._object_masscenter_lateraloffset
        h = self._object_masscenter_height


        vec_GO = np.matmul(rot_StoQ,np.array([[-R],[0],[0],[0]])) #ground to center of disk vector in s frame
        vec_OCm = np.matmul(rot_StoB,np.array([[d],[0],[h],[0]])) #disk center to center of mass
        vec_OApex = np.matmul(rot_StoB,np.array([[r],[0],[l],[0]])) #disk center to center of mass

        position_O = G_pos + vec_GO #coodinates of point O

        position_Cm = position_O + vec_OCm #position of center of mass in space frame

        position_apex = position_O + vec_OApex

        #rot_psi_diff = tfms.rotation_matrix(math.radians(self._euler.x), [0,0,1])
        #position_apex_rotated = np.matmul(rot_psi_diff,position_apex)

        return position_apex #position_apex_rotated #position_apex

    def compute_apex_position_sequence(self, dir_rock):

        R = self._object_radius

        apex_position_sequence = np.zeros((4,1))


        for theta0, alpha in zip(self._stream_theta0, self._stream_alpha):


            current_rot_psi = tfms.rotation_matrix(math.radians(self._euler.x), [0,0,1])

            init_rot = tfms.rotation_matrix(math.pi/2, [0,0,1])
            rot_StoQ = np.matmul(np.matmul(np.matmul(current_rot_psi,init_rot),tfms.rotation_matrix(theta0, [0,1,0])),tfms.rotation_matrix(dir_rock*alpha, [1,0,0]))


            [psi, theta, phi] = self.euler_from_matrix(rot_StoQ)

            rot_psi = tfms.rotation_matrix(psi, [0,0,1])
            rot_theta = tfms.rotation_matrix(theta, [0,1,0])
            rot_phi = tfms.rotation_matrix(phi, [0,0,1])

            rot_StoQ = np.matmul(np.matmul(rot_psi, init_rot),rot_theta)
            rot_StoB = np.matmul(rot_StoQ, rot_phi)

            ground_contact_position = np.matmul(np.matmul(current_rot_psi,init_rot),np.array([[R*math.cos(-1*phi)],[R*math.sin(-1*phi)],[0],[0]]))

            apex_position = self.compute_apex_position(rot_StoQ, rot_StoB, ground_contact_position)

            apex_position_sequence = np.concatenate((apex_position_sequence, apex_position), axis=1)


        apex_position_sequence = apex_position_sequence[:, 2:]
        self._diff_apex_position_sequence = apex_position_sequence - apex_position_sequence[:,0].reshape(4,1)



    def compute_potential_energy(self):
        mass_center_height = self._object_radius*math.sin(math.radians(self._euler.y)) + \
                             self._object_masscenter_height*math.cos(math.radians(self._euler.y)) - \
                             self._object_masscenter_lateraloffset*math.sin(math.radians(self._euler.y))*math.cos(math.radians(self._euler.z))

        object_pe = Float64()
        object_pe = self._object_mass*9.81*mass_center_height

        self._object_pe_pub.publish(object_pe)





if __name__ == '__main__':
    rospy.init_node("kong_pe_control", anonymous=True)

    manipulator_control = manipulation_control_law("manipulator")
    manipulator_control.intialize_arm_pose()

    rospy.loginfo("Initialized Arm Pose")
    rospy.sleep(1)

    nb_key = raw_input('Press 1 to begin rock-walk')

    try:
        number_key = int(nb_key)
    except ValueError:
        print("Invalid number")

    if number_key == 1:

        rate = rospy.Rate(50) #Rate previously 200. Change to smaller value

        dir_rock = -1 # begin with right rock
        total_rocking_steps = 3
        rock_number = 0

        while not rospy.is_shutdown():

            manipulator_control.compute_potential_energy()

            if rock_number<= total_rocking_steps:

                [dir_rock, rock_number] = manipulator_control.rock_walk(dir_rock, rock_number)


            rate.sleep()

        #plt.show()
    rospy.spin()
