import numpy as np
import rospy

from mav_msgs.msg import Actuators
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Vector3
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import yaml
from mav_planning_msgs.msg import PolynomialTrajectory4D


class Quadrotor(object):
    """Quadrotor model in ROS"""
    def __init__(self, mav_name):
        super(Quadrotor, self).__init__()
        self.name = mav_name
        self.g = 9.81
        self.k_L = 1.2e-4
        self.k_M = 0.5
        self.L = 0.0
        self.mass = 0.0
        self.rate = 0.0
        self.frequency = 50
        self.dt = 1 / 50.0

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
        self.desired_positions = np.zeros((3, 1))
        self.desired_velocities = np.zeros((3, 1))
        self.desired_acc = np.zeros((3, 1))
        self.desired_jerk = np.zeros((3, 1))
        self.desired_snap = np.zeros((3, 1))
        self.desired_angles = np.zeros((3, 1))
        self.desired_omegas = np.zeros((3, 1))
        self.desired_R_ = np.zeros((3, 3))
        self.desired_F = np.zeros((3, 1))
        self.desired_yaw = 0.0
        self.desired_d_yaw = 0.0

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

        # subscribers from ROS
        # odom
        topic_name_odom = '/' + self.name + '/ground_truth/odometry'
        self.sub_quad0_odometry = rospy.Subscriber(topic_name_odom, Odometry, self.cb_quad_odom)

        # imu
        topic_name_imu = '/' + self.name + '/ground_truth/imu'
        self.sub_quad0_imu = rospy.Subscriber(topic_name_imu, Imu, self.cb_quad_imu)

        # publisher of commands and errs
        topic_name_err_pos = '/' + self.name + '/controller/pos_err'
        topic_name_err_vel = '/' + self.name + '/controller/vel_err'
        topic_name_err_angles = '/' + self.name + '/controller/euler_err'
        topic_name_err_omegas = '/' + self.name + '/controller/omega_err'
        topic_name_err_R = '/' + self.name + '/controller/e_R'
        topic_name_euler_des = '/' + self.name + '/controller/euler_des'
        topic_name_eulers = '/' + self.name + '/controller/euler_angles'
        topic_name_pos_des = '/' + self.name + '/commander/ref_p'
        topic_name_vel_des = '/' + self.name + '/commander/ref_v'
        topic_name_actuators = '/' + self.name + '/gazebo/command/motor_speed'

        self.publisher_err_pos = rospy.Publisher(topic_name_err_pos, Vector3Stamped, queue_size=10)

        self.publisher_err_vel = rospy.Publisher(topic_name_err_vel, Vector3Stamped, queue_size=10)
        self.publisher_err_angles = rospy.Publisher(topic_name_err_angles, Vector3Stamped, queue_size=10)
        self.publisher_err_omegas = rospy.Publisher(topic_name_err_omegas, Vector3Stamped, queue_size=10)
        self.publisher_err_R = rospy.Publisher(topic_name_err_R, Vector3Stamped, queue_size=10)
        self.publisher_euler_des = rospy.Publisher(topic_name_euler_des, Vector3Stamped, queue_size=10)
        self.publisher_eulers = rospy.Publisher(topic_name_eulers, Vector3Stamped, queue_size=10)
        self.pub_desired_pos = rospy.Publisher(topic_name_pos_des, Vector3Stamped, queue_size=10)
        self.pub_desired_vel = rospy.Publisher(topic_name_vel_des, Vector3Stamped, queue_size=10)
        self.pub_actuator = rospy.Publisher(topic_name_actuators, Actuators, queue_size=10)

        # subscribers from the planner side
        self.sub_poly_trj = rospy.Subscriber('trajectory', PolynomialTrajectory4D, self.cb_trajectory)
        self.trj_poly_coeffs_x = []
        self.trj_poly_coeffs_y = []
        self.trj_poly_coeffs_z = []

        # controller
        k_pxy = 28.0
        k_pz = 28.0
        self.k_pI = 2.0
        self.K_p = np.eye(3)
        self.K_p[0, 0] = k_pxy
        self.K_p[1, 1] = k_pxy
        self.K_p[2, 2] = k_pz
        k_vxy = 12.7
        k_vz = 12.7
        self.k_vI = 2.0
        self.K_v = np.eye(3)
        self.K_v[0, 0] = k_vxy
        self.K_v[1, 1] = k_vxy
        self.K_v[2, 2] = k_vz
        self.K_omega = 22.5
        self.K_R = 40.5

        # publisher for debug:
        topic_name_motor_speed_des = '/' + self.name + '/motor_speed0_des'
        topic_name_u1_des = '/' + self.name + '/u1_des'
        topic_name_u2_des = '/' + self.name + '/u2_des'
        topic_name_u3_des = '/' + self.name + '/u3_des'
        topic_name_omega_des = '/' + self.name + '/des_omega'
        topic_name_actual_acc = '/' + self.name + '/controller/actual_acc'

        self.pub_motor_speed_des = rospy.Publisher(topic_name_motor_speed_des, Float64, queue_size=10)
        self.pub_desired_u1 = rospy.Publisher(topic_name_u1_des, Float64, queue_size=10)
        self.pub_desired_u2 = rospy.Publisher(topic_name_u2_des, Float64, queue_size=10)
        self.pub_desired_u3 = rospy.Publisher(topic_name_u3_des, Float64, queue_size=10)
        self.pub_desired_omegas = rospy.Publisher(topic_name_omega_des, Vector3Stamped, queue_size=10)
        self.pub_actual_acc = rospy.Publisher(topic_name_actual_acc, Vector3Stamped, queue_size=10)

        # commander of motor speed
        self.motor_speed = np.zeros((4, 1))
        self.isHovering = False
        self.begin_trj = True
        self.trj_received = False

        # Timer
        self.initial_time = None
        self.t = None

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    def rotationMatrixToEulerAngles(self, R):

        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def rotation_matrix_from_quaternion(quat):
        qx = quat[0]
        qy = quat[1]
        qz = quat[2]
        qw = quat[3]

        return np.array([
            [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,         1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,      1 - 2*qx*qx - 2*qy*qy]
        ])
    rotation_matrix_from_quaternion = staticmethod(rotation_matrix_from_quaternion)

    def set_initial_pos(self, x, y, z):
        self.inital_position = np.array(
            [
                [x], [y], [z]
            ]
        )
        self.desired_positions[2, 0] = z

    def pre_processing(self):
        with open('config/parameters.yaml', 'r') as stream:
            try:
                yaml_data = yaml.safe_load(stream)
                self.L = yaml_data['quad']['rotor']['length']
                mass_rotor = yaml_data['quad']['rotor']['mass']
                self.mass = yaml_data['quad']['base']['mass'] + 4 * mass_rotor
                J_base = self.inertial_dict2matrix(yaml_data['quad']['base']['inertia'])

                box_rotor = yaml_data['quad']['rotor']['box']
                J_rotor_G = self.box_inertia(box_rotor['x'], box_rotor['y'], box_rotor['z'], mass_rotor)
                J_rotors_O = [
                    self.deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor0']['origin']['x'],
                         -yaml_data['quad']['rotor0']['origin']['y'],
                         -yaml_data['quad']['rotor0']['origin']['z']], J_rotor_G, mass_rotor),

                    self.deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor1']['origin']['x'],
                         -yaml_data['quad']['rotor1']['origin']['y'],
                         -yaml_data['quad']['rotor1']['origin']['z']], J_rotor_G, mass_rotor),

                    self.deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor2']['origin']['x'],
                         -yaml_data['quad']['rotor2']['origin']['y'],
                         -yaml_data['quad']['rotor2']['origin']['z']], J_rotor_G, mass_rotor),

                    self.deplacement_moment_inertia(
                        [-yaml_data['quad']['rotor3']['origin']['x'],
                         -yaml_data['quad']['rotor3']['origin']['y'],
                         -yaml_data['quad']['rotor3']['origin']['z']], J_rotor_G, mass_rotor)
                ]
                self.J = J_base
                for i in range(4):
                    self.J = self.J + J_rotors_O[i]
            except yaml.YAMLError as exc:
                print(exc)

    def skewsymetric(input_vector):
        output_matrix = np.zeros((3, 3))
        output_matrix[0, 0] = 0
        output_matrix[1, 1] = 0
        output_matrix[2, 2] = 0
        output_matrix[0, 1] = -input_vector[2]
        output_matrix[0, 2] = input_vector[1]
        output_matrix[1, 2] = -input_vector[0]
        output_matrix[1, 0] = -output_matrix[0, 1]
        output_matrix[2, 0] = -output_matrix[0, 2]
        output_matrix[2, 1] = -output_matrix[1, 2]

    skewsymetric = staticmethod(skewsymetric)

    def inertial_dict2matrix(inertial_dict):
        inertial = np.zeros((3, 3))
        inertial[0, 0] = inertial_dict['ixx']
        inertial[0, 1] = inertial_dict['ixy']
        inertial[0, 2] = inertial_dict['ixz']
        inertial[1, 1] = inertial_dict['iyy']
        inertial[1, 2] = inertial_dict['iyz']
        inertial[2, 2] = inertial_dict['izz']
        inertial[1, 0] = inertial[0, 1]
        inertial[2, 0] = inertial[0, 2]
        inertial[2, 1] = inertial[1, 2]
        return inertial

    inertial_dict2matrix = staticmethod(inertial_dict2matrix)

    def box_inertia(x, y, z, mass):
        box_inertia = np.zeros((3, 3))
        box_inertia[0, 0] = 0.0833333 * mass * (y * y + z * z)
        box_inertia[1, 1] = 0.0833333 * mass * (x * x + z * z)
        box_inertia[2, 2] = 0.0833333 * mass * (x * x + y * y)
        return box_inertia

    box_inertia = staticmethod(box_inertia)

    def deplacement_moment_inertia(deplacement, old_inertia, mass):
        x_bar = deplacement[0]
        y_bar = deplacement[1]
        z_bar = deplacement[2]

        new_inertia = np.zeros((3, 3))
        new_inertia[0, 0] = old_inertia[0, 0] + mass * (y_bar * y_bar + z_bar * z_bar)
        new_inertia[1, 1] = old_inertia[1, 1] + mass * (x_bar * x_bar + z_bar * z_bar)
        new_inertia[2, 2] = old_inertia[2, 2] + mass * (x_bar * x_bar + y_bar * y_bar)
        new_inertia[0, 1] = old_inertia[0, 1] - mass * x_bar * y_bar
        new_inertia[1, 0] = old_inertia[0, 1] - mass * x_bar * y_bar
        new_inertia[0, 2] = old_inertia[0, 2] - mass * x_bar * z_bar
        new_inertia[2, 0] = old_inertia[0, 2] - mass * x_bar * z_bar
        new_inertia[1, 2] = old_inertia[1, 2] - mass * y_bar * z_bar
        new_inertia[2, 1] = old_inertia[1, 2] - mass * y_bar * z_bar

        return new_inertia

    deplacement_moment_inertia = staticmethod(deplacement_moment_inertia)

    def vee(Matrix):
        return np.array([
            [Matrix[2, 1]],
            [Matrix[0, 2]],
            [Matrix[1, 0]]
        ]).reshape((3, 1))

    vee = staticmethod(vee)

    def quaternion2euler(data_odom):
        return tf.transformations.euler_from_quaternion([data_odom.pose.pose.orientation.x,
                                                         data_odom.pose.pose.orientation.y,
                                                         data_odom.pose.pose.orientation.z,
                                                         data_odom.pose.pose.orientation.w])

    quaternion2euler = staticmethod(quaternion2euler)

    # callback
    def cb_quad_odom(self, data):

        self.euler_quads = np.array(self.quaternion2euler(data)).reshape((3, 1))

        self.R_ = self.rotation_matrix_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z, data.pose.pose.orientation.w])

        self.positions_quads = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y],
                                         [data.pose.pose.position.z]])

        self.acc_calculated = (np.array([[data.twist.twist.linear.x], [data.twist.twist.linear.y],
                                         [data.twist.twist.linear.z]]) - self.velocities_quads) / self.dt

        self.velocities_quads = np.array([[data.twist.twist.linear.x], [data.twist.twist.linear.y],
                                          [data.twist.twist.linear.z]])
        self.angular_vel_quads = np.array([[data.twist.twist.angular.x], [data.twist.twist.angular.y],
                                           [data.twist.twist.angular.z]])

    def cb_quad_imu(self, data):
        self.acc = np.array([
            [data.linear_acceleration.x],
            [data.linear_acceleration.y],
            [-data.linear_acceleration.z + self.g]
        ])

    # publish
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

    def publish_desired_trj(self):
        self.publish_desired_pos()
        self.publish_desired_vel()

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

    # updates
    def update_omega_err(self):
        self.e_omegas = self.angular_vel_quads - self.desired_omegas

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

    def get_desired_trj_from_poly(self):
        return

    def set_desried_trj(self):
        return

    def update_current_state(self):
        return

    def update_desired_F(self):
        self.update_pos_err()
        self.update_vel_err()

        self.desired_F = -(np.dot(self.K_p, self.e_positions) + np.multiply(self.k_pI, self.e_p_integral)) - \
                         (np.dot(self.K_v, self.e_velocities)) + np.array(
            [[0.0], [0.0], [self.mass * self.g]]) + \
                         np.multiply(self.mass, self.desired_acc)
        print ("desired F: ", self.desired_F[0, 0], self.desired_F[1, 0], self.desired_F[2, 0])
        # + np.multiply(self.k_vI, self.e_v_integral))

    def update_desired_values(self):
        return

    def motorSpeedFromU(self):
        self.motor_speed = np.dot(self.M_UtoMotor, self.u)
        print ("motor speed: ", self.motor_speed[0, 0], self.motor_speed[1, 0], self.motor_speed[2, 0], \
        self.motor_speed[3, 0])

    def send_motor_command(self):
        actuator = Actuators()
        actuator.header.stamp = rospy.Time.now()
        actuator.angular_velocities = self.motor_speed
        self.pub_actuator.publish(actuator)
        float64 = Float64()
        float64.data = self.motor_speed[0, 0]
        self.pub_motor_speed_des.publish(float64)

    def run(self):
        return

