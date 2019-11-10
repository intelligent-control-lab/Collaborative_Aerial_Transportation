#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
import numpy
import math
import numpy as np
from geometry_msgs.msg import WrenchStamped
from gym import spaces


class MultiUavState(object):

    def __init__(self, 
        max_vel, 
        min_vel, 
        max_acc,
        min_acc,
        max_jerk,
        min_jerk,
        max_snap,
        min_snap, 
        abs_max_roll, 
        abs_max_pitch,
        done_reward = -1000.0, 
        alive_reward=10.0, 
        weight_r1=1.0, 
        weight_r2=1.0, 
        weight_r3=1.0, 
        weight_r4=1.0, 
        weight_r5=1.0):

        rospy.logdebug("Starting MultiUavState Class object...")
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self.desired_world_attitude = Vector3(0.0, 0.0, 0.0)
        self._max_vel = max_vel
        self._min_vel = min_vel
        self._max_acc = max_acc
        self._min_acc = min_acc
        self._max_jerk = max_jerk
        self._min_jerk = min_jerk
        self._max_snap = max_snap
        self._min_snap = min_snap

        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._done_reward = done_reward
        self._alive_reward = alive_reward

        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3
        self._weight_r4 = weight_r4
        self._weight_r5 = weight_r5

        self._list_of_observations = ["distance_from_desired_point",
                 "base_roll",
                 "base_pitch",
                 "base_yaw",
                 "contact_force",
                 "joint_states_haa",
                 "joint_states_hfe",
                 "joint_states_kfe"]

        self.num_agents = 4
        self.payload_state = Quaternion()

        self.uav_odom_names = ['/hummingbird' + '_' + str(i) + '/' + 'ground_truth_gym/odometry' for i in range(self.num_agents)]
        self.uav_imu_names = ['/hummingbird' + '_' + str(i) + '/' + 'ground_truth_gym/imu' for i in range(self.num_agents)]
        self.uav_ft_names = ['/hummingbird' + '_' + str(i) + '/' + 'f_t' for i in range(self.num_agents)]

        self.payload_odom = '/paylaod/ground_truth/odometry'

        self.odom0 = Odometry()
        self.odom1 = Odometry()
        self.odom2 = Odometry()
        self.odom3 = Odometry()

        self.imu0 = Imu()
        self.imu1 = Imu()
        self.imu2 = Imu()
        self.imu3 = Imu()

        self.ft0 = WrenchStamped()
        self.ft1 = WrenchStamped()
        self.ft2 = WrenchStamped()
        self.ft3 = WrenchStamped()

        self.odoms = [self.odom0, self.odom1, self.odom2, self.odom3]

        self.imus = [self.imu0, self.imu1, self.imu2, self.imu3]

        self.ft = [self.ft0, self.ft1, self.ft2, self.ft3]

        self.states = np.zeros((4,20))
        self.states_concatenate = np.zeros((4,20))

        # Odom we only use it for the height detection and planar position ,
        #  because in real robots this data is not trivial.
        rospy.Subscriber(self.uav_odom_names[0], Odometry, self.odom0_callback)
        rospy.Subscriber(self.uav_odom_names[1], Odometry, self.odom1_callback)
        rospy.Subscriber(self.uav_odom_names[2], Odometry, self.odom2_callback)
        rospy.Subscriber(self.uav_odom_names[3], Odometry, self.odom3_callback)
        rospy.Subscriber(self.payload_odom, Odometry, self.payload_odom_cb)

        rospy.Subscriber(self.uav_imu_names[0], Imu, self.imu0_callback)
        rospy.Subscriber(self.uav_imu_names[1], Imu, self.imu1_callback)
        rospy.Subscriber(self.uav_imu_names[2], Imu, self.imu2_callback)
        rospy.Subscriber(self.uav_imu_names[3], Imu, self.imu3_callback)
        
        rospy.Subscriber(self.uav_ft_names[0], WrenchStamped, self.ft0_callback)
        rospy.Subscriber(self.uav_ft_names[1], WrenchStamped, self.ft1_callback)
        rospy.Subscriber(self.uav_ft_names[2], WrenchStamped, self.ft2_callback)
        rospy.Subscriber(self.uav_ft_names[3], WrenchStamped, self.ft3_callback)

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                for i in range(self.num_agents):
                    print "self.uav_imu_names[i]: ", self.uav_imu_names[i]
                    imu_data = rospy.wait_for_message(self.uav_imu_names[i], Imu, timeout=0.1)
                    self.imus[i] = imu_data
                print("Current imu_data READY")
            except:
                print("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")

        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                for i in range(self.num_agents):
                    print "self.uav_odom_names[i]: ", self.uav_odom_names[i]
                    data_pose = rospy.wait_for_message(self.uav_odom_names[i], Odometry, timeout=0.1)
                    self.odoms[i] = data_pose
                print("Current odom READY")
            except:
                print("Current odom pose not ready yet, retrying for getting robot pose")

        
        print("ALL SYSTEMS READY")

    def set_desired_world_point(self, x, y, z, roll, pitch, yaw):
        """
        Point where you want the Monoped to be
        :return:
        """
        self.desired_world_point.x = x
        self.desired_world_point.y = y
        self.desired_world_point.z = z

        self.desired_world_attitude.x = roll
        self.desired_world_attitude.y = pitch
        self.desired_world_attitude.z = yaw

    def get_uav_rpys(self):
        uav_rpys = []
        for i in range(self.num_agents):
            uav_rpys.append(tf.transformations.euler_from_quaternion(
                [self.states[i][3, 0], self.states[i][4, 0], self.states[i][5, 0], self.states[i][6, 0]]))
        return uav_rpys

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_joint_states(self):
        return self.joints_state

    def odom0_callback(self, msg):
        self.odom0 = msg
        self.odoms[0] = msg
        

    def odom1_callback(self, msg):
        self.odom1 = msg
        self.odoms[1] = msg

    def odom2_callback(self, msg):
        self.odom2 = msg
        self.odoms[2] = msg

    def odom3_callback(self, msg):
        self.odom3 = msg
        self.odoms[3] = msg

    def imu0_callback(self, msg):
        self.imu0 = msg
        self.imus[0] = msg

    def imu1_callback(self, msg):
        self.imu1 = msg
        self.imus[1] = msg

    def imu2_callback(self, msg):
        self.imu2 = msg
        self.imus[2] = msg

    def imu3_callback(self, msg):
        self.imu3 = msg
        self.imus[3] = msg

    def ft0_callback(self, msg):
        self.ft0 = msg
        self.ft[0] = msg

    def ft1_callback(self, msg):
        self.ft1 = msg
        self.ft[1] = msg

    def ft2_callback(self, msg):
        self.ft2 = msg
        self.ft[2] = msg

    def ft3_callback(self, msg):
        self.ft3 = msg
        self.ft[3] = msg

    def concatenate_states(self):
        
        for i, odom in enumerate(self.odoms):
            #print "odom.pose.pose.position.x: ", odom.pose.pose.position.x
            self.states[i][0:3] = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
            self.states[i][3:7] = np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
            self.states[i][7:10] = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
            self.states[i][10:13] = np.array([odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z])
            self.states[i][13:16] = np.array([self.imus[i].linear_acceleration.x, self.imus[i].linear_acceleration.y, self.imus[i].linear_acceleration.z])
            self.states[i][16:19] = np.array([self.ft[i].wrench.force.x, self.ft[i].wrench.force.y, self.ft[i].wrench.force.z])
            #states_concatenate = np.concatenate((states_concatenate, self.states[i]), axis=0)
        self.states_concatenate = self.states

    def payload_odom_cb(self, msg):
        self.payload_state = msg.pose.pose.orientation()

    def get_concatenate_states(self):
        return self.states_concatenate

    def uav_orientation_ok(self):
        uav_rpys = self.get_uav_rpys()
        roll_ok = self._abs_max_roll > abs(uav_rpys[0][0])
        for i in range(1, self.num_agents):
            roll_ok = roll_ok and self._abs_max_roll > abs(uav_rpys[i][0])

        pitch_ok = self._abs_max_pitch > abs(uav_rpys[0][1])
        for i in range(1, self.num_agents):
            pitch_ok = pitch_ok and self._abs_max_pitch > abs(uav_rpys[i][1])

        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def get_init_states(self):
        #self.concatenate_states()
        return self.states_concatenate


    def get_step_states(self):
        self.concatenate_states()
        return self.states_concatenate

    def calculate_total_reward(self):                                                                                                           
        return

    def calculate_reward_payload_orientation(self):
        uav_orientation_ok = self.uav_orientation_ok()
        done = not uav_orientation_ok
        if done:
            rospy.logdebug("It fell, so the reward has to be very low")
            total_reward = self._done_reward
        else:
            rospy.logdebug("Calculate normal reward because it didn't fall.")
            euler = tf.transformations.euler_from_quaternion(
                [self.payload_state.x, self.payload_state.y, self.payload_state.z, self.payload_state.w])
            reward = np.linalg.norm(euler)
        return reward, done

    def testing_loop(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_total_reward()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('monoped_state_node', anonymous=True)
    monoped_state = MonopedState(max_height=3.0,
                                 min_height=0.6,
                                 abs_max_roll=0.7,
                                 abs_max_pitch=0.7)
    monoped_state.testing_loop()
