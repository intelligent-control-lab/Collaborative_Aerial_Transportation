import numpy as np
import rospy

from mav_msgs.msg import Actuators
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml
from basic_functions import *
import os.path
import copy


class Quadrotor(object):
    """Quadrotor model in ROS"""
    def __init__(self, mav_name, index):
        super(Quadrotor, self).__init__()
        self.name = mav_name + '_' + str(index)
        self.g = 9.81
        self.k_L = 1.2e-4
        self.k_M = 0.5
        self.L = 0.0
        self.mass = 0.0
        self.rate = 0.0
        self.frequency = 100
        self.dt = 1 / 100.0

        self.payload_center = np.zeros((3, 1))

        self.pre_processing()

        self.k_T = 6.7
        self.k_phi_theta = 1.7
        self.k_psi = 1.7

        self.M_UtoMotor = np.array([
            [self.k_T, 0.0, -self.k_phi_theta, self.k_psi],
            [self.k_T, self.k_phi_theta, 0.0, -self.k_psi],
            [self.k_T, 0.0, self.k_phi_theta, self.k_psi],
            [self.k_T, -self.k_phi_theta, 0.0, -self.k_psi]
        ])

        # states of the uav
        self.inital_position = np.zeros((3, 1))
        self.positions_quads = np.zeros((3, 1))
        self.velocities_quads = np.zeros((3, 1))
        self.acc = np.zeros((3, 1))
        self.acc_calculated = np.zeros((3, 1))
        self.jerk = np.zeros((3, 1))
        self.euler_quads = np.zeros((3, 1))
        self.angular_vel_quads = np.zeros((3, 1))
        self.R_ = np.zeros((3, 3))
        self.z_B = np.zeros((1, 3))
        self.y_B = np.zeros((1, 3))
        self.x_B = np.zeros((1, 3))
        self.x_C = np.zeros((1, 3))

        # control inputs: [F, M1, M2, M3]
        self.u = np.zeros((4, 1))
        self.u[0, 0] = 10.0

        # references
        self.des_payload_position = np.zeros((3, 1))
        self.des_payload_velocity = np.zeros((3, 1))
        self.desired_positions = np.zeros((3, 1))
        self.desired_velocities = np.zeros((3, 1))
        self.desired_acceleratons = np.zeros((3, 1))
        self.desired_jerk = np.zeros((3, 1))
        self.desired_snap = np.zeros((3, 1))
        self.desired_angles = np.zeros((3, 1))
        self.desired_omegas = np.zeros((3, 1))
        self.desired_angular_acc = np.zeros((3, 1))
        self.desired_R_ = np.zeros((3, 3))
        self.desired_F = np.zeros((3, 1))
        self.desired_M = np.zeros((3, 1))
        self.desired_yaw = 0.0
        self.desired_d_yaw = 0.0
        self.desired_dd_yaw = 0.0

        self.z_B_des = np.zeros((1, 3))
        self.y_B_des = np.zeros((1, 3))
        self.x_B_des = np.zeros((1, 3))
        self.x_C_des = np.zeros((1, 3))

        # err
        self.e_positions = np.zeros((3, 1))
        self.e_p_integral = np.zeros((3, 1))
        self.e_velocities = np.zeros((3, 1))
        self.e_v_integral = np.zeros((3, 1))
        self.e_angles = np.zeros((3, 1))
        self.e_omegas = np.zeros((3, 1))
        self.e_R_ = np.zeros((3, 1))

        # self.gloabal_time = rospy.Time.now()
        # subscribers from ROS
        # odom

        topic_payload_odom = '/payload/ground_truth/odometry'
        self.sub_payload_odom = rospy.Subscriber(topic_payload_odom, Odometry, self.callback_p_odom)
        self.payload_position = np.zeros((3, 1))
        self.payload_velocity = np.zeros((3, 1))

        topic_name_odom = '/' + self.name + '/ground_truth/odometry'
        self.sub_quad0_odometry = rospy.Subscriber(topic_name_odom, Odometry, self.cb_quad_odom)

        topic_name_odom_gym = '/' + self.name + '/ground_truth_gym/odometry'
        self.pub_quad0_odometry_gym = rospy.Publisher(topic_name_odom_gym, Odometry, queue_size=1)

        # imu
        topic_name_imu = '/' + self.name + '/ground_truth/imu'
        self.sub_quad0_imu = rospy.Subscriber(topic_name_imu, Imu, self.cb_quad_imu)

        topic_name_imu_gym = '/' + self.name + '/ground_truth_gym/imu'
        self.pub_quad0_imu_gym = rospy.Publisher(topic_name_imu_gym, Imu, queue_size=1)

        # publisher of commands and errs
        topic_name_err_pos = '/' + self.name + '/controller/pos_err'
        topic_name_err_vel = '/' + self.name + '/controller/vel_err'
        # topic_name_err_angles = '/' + self.name + '/controller/euler_err'
        topic_name_err_omegas = '/' + self.name + '/controller/omega_err'
        topic_name_err_R = '/' + self.name + '/controller/e_R'
        topic_name_euler_des = '/' + self.name + '/controller/euler_des'
        topic_name_eulers = '/' + self.name + '/controller/euler_angles'
        topic_name_pos_des = '/' + self.name + '/commander/ref_p'
        topic_name_vel_des = '/' + self.name + '/commander/ref_v'
        topic_name_acc_des = '/' + self.name + '/commander/ref_a'
        topic_name_jerk_des = '/' + self.name + '/commander/ref_j'
        topic_name_snap_des = '/' + self.name + '/commander/ref_snap'
        topic_name_actuators = '/' + self.name + '/gazebo/command/motor_speed'
        

        # payload desired_trj
        topic_name_ref_pos_payload = '/payload/ref_pos'
        topic_name_ref_vel_payload = '/payload/ref_vel'
        self.pub_payload_desired_pos = rospy.Publisher(topic_name_ref_pos_payload, Vector3Stamped, queue_size=1)
        self.pub_payload_desired_vel = rospy.Publisher(topic_name_ref_vel_payload, Vector3Stamped, queue_size=1)


        self.publisher_err_pos = rospy.Publisher(topic_name_err_pos, Vector3Stamped, queue_size=1)
        self.publisher_err_vel = rospy.Publisher(topic_name_err_vel, Vector3Stamped, queue_size=1)
        # self.publisher_err_angles = rospy.Publisher(topic_name_err_angles, Vector3Stamped, queue_size=10)
        self.publisher_err_omegas = rospy.Publisher(topic_name_err_omegas, Vector3Stamped, queue_size=1)
        self.publisher_err_R = rospy.Publisher(topic_name_err_R, Vector3Stamped, queue_size=1)
        self.publisher_euler_des = rospy.Publisher(topic_name_euler_des, Vector3Stamped, queue_size=1)
        self.publisher_eulers = rospy.Publisher(topic_name_eulers, Vector3Stamped, queue_size=1)
        self.pub_desired_pos = rospy.Publisher(topic_name_pos_des, Vector3Stamped, queue_size=1)
        self.pub_desired_vel = rospy.Publisher(topic_name_vel_des, Vector3Stamped, queue_size=1)
        self.pub_desired_acc = rospy.Publisher(topic_name_acc_des, Vector3Stamped, queue_size=1)
        self.pub_desired_jerk = rospy.Publisher(topic_name_jerk_des, Vector3Stamped, queue_size=1)
        self.pub_desired_snap = rospy.Publisher(topic_name_snap_des, Vector3Stamped, queue_size=1)
        
        self.pub_actuator = rospy.Publisher(topic_name_actuators, Actuators, queue_size=1)

        # subscribers from the planner side
        # self.sub_poly_trj = rospy.Subscriber('trajectory', PolynomialTrajectory4D, self.cb_trajectory)
        self.trj_poly_coeffs_x = []
        self.trj_poly_coeffs_y = []
        self.trj_poly_coeffs_z = []

        # controller parameters for controlling a single robot
        k_pxy = 47.0
        k_pz = 47.0
        k_vxy = 60.7
        k_vz = 60.7
        k_omega = 30.5
        k_R = 193.5

        self.k_pI = 2.0
        self.K_p = np.eye(3)
        self.K_p[0, 0] = k_pxy
        self.K_p[1, 1] = k_pxy
        self.K_p[2, 2] = k_pz

        self.k_vI = 2.0
        self.K_v = np.eye(3)
        self.K_v[0, 0] = k_vxy
        self.K_v[1, 1] = k_vxy
        self.K_v[2, 2] = k_vz
        self.K_omega = k_omega
        self.K_R = k_R

        # publisher for debug:
        topic_name_motor_speed_des = '/' + self.name + '/motor_speed0_des'
        # topic_name_u1_des = '/' + self.name + '/u1_des'
        # topic_name_u2_des = '/' + self.name + '/u2_des'
        # topic_name_u3_des = '/' + self.name + '/u3_des'
        topic_name_omega_des = '/' + self.name + '/des_omega'
        topic_name_actual_acc = '/' + self.name + '/controller/actual_acc'


        self.pub_motor_speed_des = rospy.Publisher(topic_name_motor_speed_des, Float64, queue_size=1)
        # self.pub_desired_u1 = rospy.Publisher(topic_name_u1_des, Float64, queue_size=1)
        # self.pub_desired_u2 = rospy.Publisher(topic_name_u2_des, Float64, queue_size=1)
        # self.pub_desired_u3 = rospy.Publisher(topic_name_u3_des, Float64, queue_size=1)
        self.pub_desired_omegas = rospy.Publisher(topic_name_omega_des, Vector3Stamped, queue_size=1)
        self.pub_actual_acc = rospy.Publisher(topic_name_actual_acc, Vector3Stamped, queue_size=1)


        # commander of motor speed
        self.motor_speed = np.zeros((4, 1))
        self.isHovering = False
        self.begin_trj = True
        self.trj_received = False

        # Timer
        self.initial_time = None
        self.t = None

        self.odom_gym = None
        self.imu_gym = None

        # print "topic to wait: ", topic_name_odom
        # odom = rospy.wait_for_message(topic_name_odom, Odometry)
        # print "topic to wait: ", topic_name_imu
        # imu = rospy.wait_for_message(topic_name_imu, Imu)

        # self.check_publishers_connection()

    def callback_p_odom(self, data):
        self.payload_position = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y], [data.pose.pose.position.z]])
        self.payload_velocity = np.array([[data.twist.twist.linear.x], [data.twist.twist.linear.y], [data.twist.twist.linear.z]])


    def set_initial_pos(self, x, y, z):
        self.inital_position = np.array(
            [
                [x], [y], [z]
            ]
        )
        self.desired_positions[2, 0] = z

    def pre_processing(self):
        current_directory = os.path.dirname(__file__)
        # parent_directory = os.path.split(current_directory)[0]
        file_path = os.path.join(current_directory, 'config', 'parameters.yaml')
        with open(file_path, 'r') as stream:
            try:
                yaml_data = yaml.safe_load(stream)
                self.L = yaml_data['quad']['rotor']['length']
                mass_rotor = yaml_data['quad']['rotor']['mass']
                self.mass = yaml_data['quad']['base']['mass'] + 4 * mass_rotor + 3 * 0.025 #mass spherical joints
                J_base = inertial_dict2matrix(yaml_data['quad']['base']['inertia'])

                box_rotor = yaml_data['quad']['rotor']['box']
                J_rotor_G = box_inertia(box_rotor['x'], box_rotor['y'], box_rotor['z'], mass_rotor)
                J_rotors_O = [
                    deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor0']['origin']['x'],
                         -yaml_data['quad']['rotor0']['origin']['y'],
                         -yaml_data['quad']['rotor0']['origin']['z']], J_rotor_G, mass_rotor),

                    deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor1']['origin']['x'],
                         -yaml_data['quad']['rotor1']['origin']['y'],
                         -yaml_data['quad']['rotor1']['origin']['z']], J_rotor_G, mass_rotor),

                    deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor2']['origin']['x'],
                         -yaml_data['quad']['rotor2']['origin']['y'],
                         -yaml_data['quad']['rotor2']['origin']['z']], J_rotor_G, mass_rotor),

                    deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor3']['origin']['x'],
                         -yaml_data['quad']['rotor3']['origin']['y'],
                         -yaml_data['quad']['rotor3']['origin']['z']], J_rotor_G, mass_rotor)
                ]
                self.payload_center[2, 0] = yaml_data['payload']['base']['box']['z'] / 2.0
                self.J = J_base
                for i in range(4):
                    self.J = self.J + J_rotors_O[i]
            except yaml.YAMLError as exc:
                print(exc)

    def cb_quad_odom(self, data):
        #print "callback inside!!!"
        if data.twist.twist.linear.x != data.twist.twist.linear.x:
            print "aaaaaaaaaaaaaaaaaaaaaaaaaaa nan velocities!"
            return

        self.euler_quads = np.array(quaternion2euler([data.pose.pose.orientation.x,
                                                     data.pose.pose.orientation.y,
                                                     data.pose.pose.orientation.z,
                                                     data.pose.pose.orientation.w])).reshape((3, 1))

        self.R_ = rotation_matrix_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z, data.pose.pose.orientation.w])

        # print "self.positions_quads: ", self.positions_quads
        self.positions_quads = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y],
                                         [data.pose.pose.position.z]])

        self.acc_calculated = (np.array([[data.twist.twist.linear.x], [data.twist.twist.linear.y],
                                         [data.twist.twist.linear.z]]) - self.velocities_quads) / self.dt

        # print "data.twist.twist.linear.x: ", data.twist.twist.linear.x

        # print "self.velocities_quads inside msg: ", self.velocities_quads
        self.angular_vel_quads = np.array([[data.twist.twist.angular.x], [data.twist.twist.angular.y],
                                           [data.twist.twist.angular.z]])
        
        self.velocities_quads = np.array([[data.twist.twist.linear.x], [data.twist.twist.linear.y],
                                          [data.twist.twist.linear.z]])

        self.update_pos_err()
        self.update_vel_err()

        self.odom_gym = copy.copy(data)
        self.publish_gym_odom()
        # time_now = data.header.stamp
        # duration = (time_now - self.gloabal_time).to_sec()
        # print "duration velocities: ", duration
        # self.gloabal_time = time_now
        # self.x_B = self.R_[:, 0].reshape((1, 3))
        # self.y_B = self.R_[:, 1].reshape((1, 3))
        # self.z_B = self.R_[:, 2].reshape((1, 3))


    def publish_gym_odom(self):
        if not self.odom_gym is None:
            self.pub_quad0_odometry_gym.publish(self.odom_gym)
        else:
            print "gym odom state is None!!"

    def publish_gym_imu(self):
        if not self.imu_gym is None:
            self.pub_quad0_imu_gym.publish(self.imu_gym)
        else:
            print "gym imu state is None!!"

    def cb_quad_imu(self, data):
        self.acc = np.array([
            [data.linear_acceleration.x],
            [data.linear_acceleration.y],
            [-data.linear_acceleration.z + self.g]
        ])

        self.imu_gym = copy.copy(data)
        self.publish_gym_imu()
        

    # publish
    def publish_desired_payload_pos(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.des_payload_position[0, 0], self.des_payload_position[1, 0], \
                                  self.des_payload_position[2, 0])
        self.pub_payload_desired_pos.publish(pub_vec3)

    def publish_desired_payload_vel(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.des_payload_velocity[0, 0], self.des_payload_velocity[1, 0], \
                                  self.des_payload_velocity[2, 0])
        self.pub_payload_desired_vel.publish(pub_vec3)

    def publish_desired_pos(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.desired_positions[0, 0], self.desired_positions[1, 0], \
                                  self.desired_positions[2, 0])
        self.pub_desired_pos.publish(pub_vec3)

    def publish_desired_vel(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.desired_velocities[0, 0], self.desired_velocities[1, 0], \
                                  self.desired_velocities[2, 0])
        self.pub_desired_vel.publish(pub_vec3)

    def publish_desired_acc(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.desired_acceleratons[0, 0], self.desired_acceleratons[1, 0], \
                                  self.desired_acceleratons[2, 0])
        self.pub_desired_acc.publish(pub_vec3)

    def publish_desired_jerk(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.desired_jerk[0, 0], self.desired_jerk[1, 0], \
                                  self.desired_jerk[2, 0])
        self.pub_desired_jerk.publish(pub_vec3)

    def publish_desired_snap(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.desired_snap[0, 0], self.desired_snap[1, 0], \
                                  self.desired_snap[2, 0])
        self.pub_desired_snap.publish(pub_vec3)

    def publish_desired_trj(self):
        self.publish_desired_pos()
        self.publish_desired_vel()
        self.publish_desired_payload_pos()
        self.publish_desired_payload_vel()
        # self.publish_desired_acc()
        # self.publish_desired_jerk()
        # self.publish_desired_snap()

        # self.publish_gym_states()

    def publish_err(self):
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.e_positions[0, 0], self.e_positions[1, 0], self.e_positions[2, 0])
        self.publisher_err_pos.publish(pub_vec3)

        pub_vec3.vector = Vector3(self.e_velocities[0, 0], self.e_velocities[1, 0],
                                  self.e_velocities[2, 0])
        self.publisher_err_vel.publish(pub_vec3)

        pub_vec3.vector = Vector3(self.e_omegas[0, 0], self.e_omegas[1, 0], self.e_omegas[2, 0])
        self.publisher_err_omegas.publish(pub_vec3)
        del pub_vec3


    # updates
    def update_omega_err(self):
        # self.e_omegas = self.angular_vel_quads - self.desired_omegas
        self.e_omegas = self.angular_vel_quads - np.dot(
            np.dot(self.R_.T, self.desired_R_), self.desired_omegas)

    def update_pos_err(self):
        self.e_positions = self.positions_quads - self.desired_positions
        self.e_p_integral = np.add(self.e_p_integral, np.multiply(self.e_positions, self.dt))
        self.e_p_integral[np.where(self.e_p_integral >= 2.0)] = 2.0
        self.e_p_integral[np.where(self.e_p_integral <= -2.0)] = -2.0

    def update_vel_err(self):
        self.e_velocities = self.velocities_quads - self.desired_velocities

    def update_R_err(self):
        return

    def update_M(self):
        self.update_omega_err()
        self.update_R_err()
        self.u[1:4] = - np.multiply(self.K_R, self.e_R_) - np.multiply(self.K_omega, self.e_omegas)

    # def get_desired_trj_from_poly(self):
    #     return

    # def set_desried_trj(self):
    #     return

    # def update_current_state(self):
    #     return

    def update_desired_F(self):
        # self.update_pos_err()
        # self.update_vel_err()

        self.desired_F = -(np.dot(self.K_p, self.e_positions) + np.multiply(self.k_pI, self.e_p_integral)) - \
                         (np.dot(self.K_v, self.e_velocities)) + np.array(
            [[0.0], [0.0], [self.mass * self.g]]) + \
                         np.multiply(self.mass, self.desired_acceleratons)
        print "desired F: ", self.desired_F[0, 0], self.desired_F[1, 0], self.desired_F[2, 0]

    # def update_desired_values(self):
    #     return

    def offset_thrust(self, delta_f):
        self.u[0] = self.u[0] + delta_f

    def motorSpeedFromU(self):
        self.motor_speed = np.dot(self.M_UtoMotor, self.u)
        # print ("motor speeds: ", self.motor_speed[0, 0], self.motor_speed[1, 0], self.motor_speed[2, 0], \
        # self.motor_speed[3, 0])

    def set_desired_motor_speed(self, motor_speed):
        self.motor_speed = motor_speed

    def send_motor_command(self):
        actuator = Actuators()
        actuator.header.stamp = rospy.Time.now()
        # print ("motor_speed: ", self.motor_speed)
        actuator.angular_velocities = self.motor_speed
        self.pub_actuator.publish(actuator)
        float64 = Float64()
        float64.data = self.motor_speed[0, 0]/1000.0
        self.pub_motor_speed_des.publish(float64)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self.pub_actuator.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.loginfo("No susbribers to _roll_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.loginfo("_base_pub Publisher Connected")

        rospy.loginfo("All Publishers READY")

    def run(self):
        return

