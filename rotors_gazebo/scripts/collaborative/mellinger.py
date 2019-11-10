from quadrotor import Quadrotor
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Vector3
import numpy as np
import rospy
from polynomialTrjNonlinear.poly_trajectory3D import PolyTrajectory3D


class Mellinger(Quadrotor):
    """Mellinger Controller with Polynomial Trajectory Generation"""

    def __init__(self, mav_name, x, y, z):
        Quadrotor.__init__(self, mav_name)

        self.k_T = 6.7
        self.k_phi_theta = 1.7
        self.k_psi = 1.7
        self.set_initial_pos(x, y, z)
        self.M_UtoMotor = np.array([
            [self.k_T,        0.0,         -self.k_phi_theta,    self.k_psi],
            [self.k_T,    self.k_phi_theta,        0.0,        -self.k_psi],
            [self.k_T,        0.0,          self.k_phi_theta,   self.k_psi],
            [self.k_T,   -self.k_phi_theta,       0.0,         -self.k_psi]
        ])

        # controller parameters for controlling a single robot
        k_pxy = 47.0
        k_pz = 47.0
        k_vxy = 19.7
        k_vz = 19.7
        k_omega = 11.5
        k_R = 85.5

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

        # Timer
        self.initial_time = None
        self.t = None
        self.hover_begin_time = None
        self.hover_duration = 0.0
        # Polynomial trajectory planner
        self.isPolynomialSolved = False
        self.planner = PolyTrajectory3D(offset=np.array([x, y, z]))
        self.trj_xyz = np.zeros((3, 1))
        self.trj_v = np.zeros((3, 1))
        self.trj_x_list = []
        self.trj_y_list = []
        self.trj_z_list = []

        self.trj_vx_list = []
        self.trj_vy_list = []
        self.trj_vz_list = []

    def cb_trajectory(self, data):
        print (data.segments[0].x)
        self.trj_poly_coeffs_x = data.segments[0].x
        self.trj_poly_coeffs_y = data.segments[0].y
        self.trj_poly_coeffs_z = data.segments[0].z
        self.trj_received = True

    def set_control_gains(self, k_pxy=14.0, k_pz=55.0, k_vxy=10.7, k_vz=10.7, k_omega=22.5, k_R=40.5):
        self.K_p[0, 0] = k_pxy
        self.K_p[1, 1] = k_pxy
        self.K_p[2, 2] = k_pz

        self.K_v[0, 0] = k_vxy
        self.K_v[1, 1] = k_vxy
        self.K_v[2, 2] = k_vz

        self.K_omega = k_omega
        self.K_R = k_R

    # updates
    def update_offset(self):
        self.planner.set_offset(self.positions_quads[0], self.positions_quads[1], self.positions_quads[2])

    def update_omega_err(self):
        self.e_omegas = self.angular_vel_quads - self.desired_omegas

    def update_R_err(self):
        e_R = np.dot(self.desired_R_.transpose(), self.R_) - \
                             np.dot(self.R_.transpose(), self.desired_R_)
        self.e_R_ = self.vee(e_R) / 2.0

        vec3_e_R = Vector3Stamped()
        vec3_e_R.header.stamp = rospy.Time.now()
        euler_des = self.rotationMatrixToEulerAngles(self.desired_R_)
        vec3_e_R.vector = Vector3(euler_des[0], euler_des[1], euler_des[2])
        self.publisher_euler_des.publish(vec3_e_R)

        vec3_e_R = Vector3Stamped()
        vec3_e_R.header.stamp = rospy.Time.now()
        vec3_e_R.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        self.publisher_err_R.publish(vec3_e_R)

        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.euler_quads[0, 0], self.euler_quads[1, 0], self.euler_quads[2, 0])
        self.publisher_eulers.publish(pub_vec3)

        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        self.publisher_err_angles.publish(pub_vec3)

    def update_M(self):
        self.update_omega_err()
        self.update_R_err()
        self.u[1:4] = - np.multiply(self.K_R, self.e_R_) - np.multiply(self.K_omega, self.e_omegas)

    def get_desired_trj_from_poly(self):
        if not self.trj_received:
            return
        if self.begin_trj:
            self.begin_trj = False
            self.t = self.t.to_sec() + self.dt
        else:
            self.t += self.dt
        print ("t: ", self.t)
        ts = np.array([pow(self.t, 9), pow(self.t, 8), pow(self.t, 7), pow(self.t, 6), pow(self.t,5),\
                       pow(self.t, 4), pow(self.t, 3), pow(self.t, 2), pow(self.t, 1), self.t])

        dt_s = np.array([9 * pow(self.t, 8), 8 * pow(self.t, 7), 7 * pow(self.t, 6), 6 * pow(self.t, 5),\
                         5 * pow(self.t, 4), 4 * pow(self.t, 3), 3 * pow(self.t, 2), 2 * pow(self.t, 1), 1, 0])

        self.desired_positions[0, 0] = np.dot(self.trj_poly_coeffs_x, ts)
        self.desired_positions[1, 0] = np.dot(self.trj_poly_coeffs_y, ts)
        self.desired_positions[2, 0] = np.dot(self.trj_poly_coeffs_z, ts)

        d_trj_poly_coeffs_x = np.concatenate(([0.0], self.trj_poly_coeffs_x[1:]), 0)
        d_trj_poly_coeffs_y = np.concatenate(([0.0], self.trj_poly_coeffs_y[1:]), 0)
        d_trj_poly_coeffs_z = np.concatenate(([0.0], self.trj_poly_coeffs_z[1:]), 0)

        self.desired_velocities[0, 0] = np.dot(d_trj_poly_coeffs_x, dt_s)
        self.desired_velocities[1, 0] = np.dot(d_trj_poly_coeffs_y, dt_s)
        self.desired_velocities[2, 0] = np.dot(d_trj_poly_coeffs_z, dt_s)

        self.publish_desired_trj()

    def set_hover_des(self, target_height):
        if not self.isHovering:
            if self.positions_quads[2, 0] > self.inital_position[2, 0] + target_height:
                self.isHovering = True
                self.hover_begin_time = rospy.Time.now()
            else:
                self.desired_positions[0, 0] = self.inital_position[0, 0]
                self.desired_positions[1, 0] = self.inital_position[1, 0]

                if self.desired_velocities[2, 0] < 0.5:
                    self.desired_velocities[2, 0] = self.desired_velocities[2, 0] + 0.01
                else:
                    self.desired_velocities = np.array([[0.0], [0.0], [0.5]])
                self.desired_positions[2, 0] = self.desired_positions[2, 0] + self.desired_velocities[2, 0] * self.dt
        else:
            self.hover_duration = rospy.Time.now().to_sec() - self.hover_begin_time.to_sec()
            print self.name, 'hovering duration: ', self.hover_duration
            if self.desired_velocities[2, 0] > 0.0:
                self.desired_velocities[2, 0] = self.desired_velocities[2, 0] - 0.01
                self.desired_positions[2, 0] = self.desired_positions[2, 0] + self.desired_velocities[2, 0] * self.dt
            else:
                self.desired_velocities[2, 0] = 0.0
        self.publish_desired_trj()

    def set_desried_trj(self):
        return

    def update_current_state(self):
        # actual values
        acc_vec = Vector3Stamped()
        acc_vec.header.stamp = rospy.Time.now()
        acc_vec.vector = Vector3(self.acc[0, 0], self.acc[1, 0], self.acc[2, 0])
        self.pub_actual_acc.publish(acc_vec)

        t = self.acc + np.array([[0.0], [0.0], [self.g]])

        self.z_B = (t / np.linalg.norm(t)).transpose()
        self.x_C = np.array([np.cos(self.euler_quads[2, 0]), np.sin(self.euler_quads[2, 0]), 0.0])
        self.y_B = np.cross(self.z_B, self.x_C) / np.linalg.norm(np.cross(self.z_B, self.x_C))
        self.x_B = np.cross(self.y_B, self.z_B)

    def update_desired_F(self):
        self.update_pos_err()
        self.update_vel_err()

        self.desired_F = -(np.dot(self.K_p, self.e_positions) + np.multiply(self.k_pI, self.e_p_integral)) - \
                          (np.dot(self.K_v, self.e_velocities)) + np.array([[0.0], [0.0], [self.mass * self.g]]) + \
                           np.multiply(self.mass, self.desired_acc)

    def update_desired_values(self):
        # desired values
        self.update_desired_F()
        self.u[0, 0] = np.dot(self.z_B, self.desired_F)

        float_u = Float64()
        float_u.data = self.u[0, 0]/10.0 # for better visualization
        self.pub_desired_u1.publish(float_u)

        self.z_B_des = (self.desired_F / np.linalg.norm(self.desired_F)).transpose()
        self.x_C_des = np.array([np.cos(self.desired_yaw), np.sin(self.desired_yaw), 0.0])
        self.y_B_des = np.cross(self.z_B_des, self.x_C_des) / \
                       np.linalg.norm(np.cross(self.z_B_des, self.x_C_des))
        self.x_B_des = np.cross(self.y_B_des, self.z_B_des)
        self.desired_R_ = np.concatenate((self.x_B_des.transpose(), self.y_B_des.transpose(),
                                          self.z_B_des.transpose()), axis=1)

        h_omega = self.mass / self.u[0, 0] * (self.jerk - (np.multiply(np.dot(self.z_B, self.jerk),
                                                                       self.z_B.transpose())))
        self.desired_omegas[0, 0] = -np.dot(self.y_B_des, h_omega)
        self.desired_omegas[1, 0] = np.dot(self.x_B_des, h_omega)
        self.desired_omegas[2, 0] = np.dot(self.z_B_des, np.array([[0.0], [0.0], [self.desired_d_yaw]]))

        self.update_M()

        float_u.data = self.u[1, 0] / 10.0  # for better visualization
        self.pub_desired_u2.publish(float_u)
        float_u.data = self.u[2, 0] / 10.0  # for better visualization
        self.pub_desired_u3.publish(float_u)

        vec3_omega_des = Vector3Stamped()
        vec3_omega_des.header.stamp = rospy.Time.now()
        vec3_omega_des.vector = Vector3(self.desired_omegas[0, 0], self.desired_omegas[1, 0],
                                        self.desired_omegas[2, 0])
        self.pub_desired_omegas.publish(vec3_omega_des)

    def motorSpeedFromU(self):
        self.motor_speed = np.dot(self.M_UtoMotor, self.u)

    def multiply_motor_speed(self, k):
        self.motor_speed = np.multiply(self.motor_speed, k)

    def set_constrain(self, dimention='x', orders=None, zeroValues=None, endVlues=None):
        if dimention == 'x':
            self.planner.poly_x.set_constrains(orders, zeroValues, endVlues)
        elif dimention == 'y':
            self.planner.poly_y.set_constrains(orders, zeroValues, endVlues)
        elif dimention == 'z':
            self.planner.poly_z.set_constrains(orders, zeroValues, endVlues)

    def solve_poly3d(self):
        self.planner.solve_traj_xyz(sample_frequency=self.frequency)
        self.isPolynomialSolved = True

    def load_trj_lists(self, dimension='xyz'):
        self.trj_xyz = self.planner.get_3d_trj()
        self.trj_v = self.planner.get_3d_velocities()

        if dimension == 'xyz':
            self.trj_x_list = self.trj_xyz[:, 0].tolist()
            self.trj_y_list = self.trj_xyz[:, 1].tolist()
            self.trj_z_list = self.trj_xyz[:, 2].tolist()

            self.trj_vx_list = self.trj_v[:, 0].tolist()
            self.trj_vy_list = self.trj_v[:, 1].tolist()
            self.trj_vz_list = self.trj_v[:, 2].tolist()
        elif dimension == 'x':
            self.trj_x_list = self.trj_xyz[:, 0].tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.trj_xyz[:, 0].shape)).tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.trj_xyz[:, 0].shape)).tolist()

            self.trj_vx_list = self.trj_v[:, 0].tolist()
            self.trj_vy_list = np.zeros(self.trj_v[:, 0].shape).tolist()
            self.trj_vz_list = np.zeros(self.trj_v[:, 0].shape).tolist()

        elif dimension == 'y':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.trj_xyz[:, 1].shape)).tolist()
            self.trj_y_list = self.trj_xyz[:, 1].tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.trj_xyz[:, 1].shape)).tolist()

            self.trj_vx_list = np.zeros(self.trj_v[:, 1].shape).tolist()
            self.trj_vy_list = self.trj_v[:, 1].tolist()
            self.trj_vz_list = np.zeros(self.trj_v[:, 1].shape).tolist()

        elif dimension == 'z':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.trj_xyz[:, 1].shape)).tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.trj_xyz[:, 1].shape)).tolist()
            self.trj_z_list = self.trj_xyz[:, 2].tolist()

            self.trj_vx_list = np.zeros(self.trj_v[:, 2].shape).tolist()
            self.trj_vy_list = np.zeros(self.trj_v[:, 2].shape).tolist()
            self.trj_vz_list = self.trj_v[:, 2].tolist()

    def trj_from_poly_coeffs(self, t):
        """
        calculate the desired trajectory given a time point
        :param t: time (s)
        """
        # coeffs = self.

    def publish_poly3d_point(self, xyz_arr, v_xyz_arr):
        self.desired_positions = xyz_arr.reshape((3, 1))
        self.desired_velocities = v_xyz_arr.reshape((3, 1))
        self.publish_desired_trj()

    def publish_poly3d_trj(self):
        if self.isPolynomialSolved:
            if len(self.trj_x_list) == 1:
                trj_tmp = np.array([self.trj_x_list[0], self.trj_y_list[0], self.trj_z_list[0]])
                trj_v_tmp = np.array([self.trj_vx_list[0], self.trj_vy_list[0], self.trj_vz_list[0]])
            else:
                trj_tmp = np.array([self.trj_x_list.pop(0), self.trj_y_list.pop(0), self.trj_z_list.pop(0)])
                trj_v_tmp = np.array([self.trj_vx_list.pop(0), self.trj_vy_list.pop(0), self.trj_vz_list.pop(0)])
            self.publish_poly3d_point(trj_tmp, trj_v_tmp)
        else:
            print "Polynomial has not been solved yet! Please solve the poly coefficients first!"

    def test_follow_trj(self):
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(50)
        rospy.sleep(3.0)
        while not rospy.is_shutdown():
            self.get_desired_trj_from_poly()
            self.publish_err()
            self.update_current_state()
            self.update_desired_values()
            self.motorSpeedFromU()
            # self.send_motor_command()
            self.rate.sleep()

    def hover(self, target_height):
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)
        while not rospy.is_shutdown():
            self.set_hover_des(target_height)
            self.publish_err()
            self.update_current_state()
            self.update_desired_values()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def run(self):
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)
        self.planner.set_segment_time(segment_times=[6.0])
        self.solve_poly3d()
        self.planner.add_offset()
        self.load_trj_lists()
        while not rospy.is_shutdown():
            self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_values()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def hover_and_trj_xy(self, dimension='x'):
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)
        self.planner.set_segment_time(segment_times=[6.0])
        self.solve_poly3d()
        # self.planner.add_offset()
        self.load_trj_lists(dimension='xyz')
        while not rospy.is_shutdown():
            if self.hover_duration < 5.0:
                self.set_hover_des(target_height=1.5)
            else:
                if not self.planner.offset_added:
                    print "hovering finished, going into the next phase..."
                    self.load_trj_lists(dimension=dimension)
                    self.planner.add_offset()
                    print "offset added"
                self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_values()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()





