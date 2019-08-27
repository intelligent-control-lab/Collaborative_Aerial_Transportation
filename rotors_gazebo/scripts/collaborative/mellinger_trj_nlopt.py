from quadrotor import Quadrotor

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Vector3

from polynomialTrjNonlinear.Optimizer_nonlinear import PolynomialOptNonlinear
from polynomialTrjNonlinear.vertex import Vertex
from basic_functions import *


class Mellinger(Quadrotor):
    """
    Mellinger Controller with Polynomial Trajectory Generation
    Using nonlinear optimizations.
    """

    def __init__(self, mav_name, index, x, y, z, dimension=3, N=10):
        Quadrotor.__init__(self, mav_name, index)

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
        k_vxy = 60.7
        k_vz = 60.7
        k_omega = 25.5
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
        self.NL_planner = PolynomialOptNonlinear(N=N, dimension=dimension)

        self.trj_xyz = np.zeros((3, 1))
        self.trj_v = np.zeros((3, 1))
        self.trj_acc = np.zeros((3, 1))
        self.trj_jerk = np.zeros((3, 1))
        self.trj_snap = np.zeros((3, 1))

        self.trj_x_list = []
        self.trj_y_list = []
        self.trj_z_list = []

        self.trj_vx_list = []
        self.trj_vy_list = []
        self.trj_vz_list = []

        self.trj_accx_list = []
        self.trj_accy_list = []
        self.trj_accz_list = []

        self.trj_jerkx_list = []
        self.trj_jerky_list = []
        self.trj_jerkz_list = []

        self.trj_snapx_list = []
        self.trj_snapy_list = []
        self.trj_snapz_list = []

        self.offset_added = False

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
        self.NL_planner.init_offset(self.positions_quads.T)
        self.offset_added = True

    def update_omega_err(self):
        self.e_omegas = self.angular_vel_quads - np.dot(
            np.dot(self.R_.T, self.desired_R_), self.desired_omegas)
        # self.e_omegas = self.angular_vel_quads - self.desired_omegas
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.e_omegas[0, 0], self.e_omegas[1, 0], self.e_omegas[2, 0])
        self.publisher_err_omegas.publish(pub_vec3)

    def update_R_err(self):
        e_R = np.dot(self.desired_R_.transpose(), self.R_) - \
                             np.dot(self.R_.transpose(), self.desired_R_)
        self.e_R_ = vee(e_R) / 2.0

        vec3_euler_des = Vector3Stamped()
        vec3_euler_des.header.stamp = rospy.Time.now()

        euler_des = rotationMatrixToEulerAngles(self.desired_R_)
        vec3_euler_des.vector = Vector3(euler_des[0], euler_des[1], euler_des[2])
        self.publisher_euler_des.publish(vec3_euler_des)

        vec3_e_R = Vector3Stamped()
        vec3_e_R.header.stamp = rospy.Time.now()
        vec3_e_R.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        self.publisher_err_R.publish(vec3_e_R)

        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.euler_quads[0, 0], self.euler_quads[1, 0], self.euler_quads[2, 0])
        self.publisher_eulers.publish(pub_vec3)

        # pub_vec3 = Vector3Stamped()
        # pub_vec3.header.stamp = rospy.Time.now()
        # pub_vec3.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        # self.publisher_err_angles.publish(pub_vec3)

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

    def construct_vertices(self, re_goal_position):
        vertices = []
        vertex0 = Vertex(dimension=3, index=0)
        vertex0.makeStartOrEnd(position=self.positions_quads, up_to_order=4)

        vertex1 = Vertex(dimension=3, index=1)
        vertex1.makeStartOrEnd(position=re_goal_position, up_to_order=4)

        vertices.append(vertex0)
        vertices.append(vertex1)
        self.NL_planner.setupFromVertices(vertices=vertices)

        self.NL_planner.add_max_vel(2.0)
        self.NL_planner.add_max_acc(1.0)

        self.optimize()

        self.getPlanUpToSnap(frequency=50)

    def optimize(self):
        result = self.NL_planner.optimizeTimeAndFreeConstraints()
        self.isPolynomialSolved = True

    def getPlanUpToSnap(self, frequency):
        self.NL_planner.linear_opt.get_d_trajectory(order=0, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=1, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=2, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=3, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=4, sample_frequency=frequency)

    def update_current_state(self):
        # actual values
        acc_vec = Vector3Stamped()
        acc_vec.header.stamp = rospy.Time.now()
        acc_vec.vector = Vector3(self.acc[0, 0], self.acc[1, 0], self.acc[2, 0])
        self.pub_actual_acc.publish(acc_vec)

        self.x_B = self.R_[:, 0].reshape((1, 3))
        self.y_B = self.R_[:, 1].reshape((1, 3))
        self.z_B = self.R_[:, 2].reshape((1, 3))

        # self.update_c()

        # t = self.acc + np.array([[0.0], [0.0], [self.g]])
        #
        # self.z_B = (t / np.linalg.norm(t)).transpose()
        # self.x_C = np.array([np.cos(self.euler_quads[2, 0]), np.sin(self.euler_quads[2, 0]), 0.0])
        # self.y_B = np.cross(self.z_B, self.x_C) / np.linalg.norm(np.cross(self.z_B, self.x_C))
        # self.x_B = np.cross(self.y_B, self.z_B)

    def update_desired_inputs(self):
        self.update_desired_F()
        self.update_desired_values()
        self.update_desired_M()

    def update_desired_F(self):
        self.update_pos_err()
        self.update_vel_err()

        self.desired_F = -(np.dot(self.K_p, self.e_positions) + np.multiply(self.k_pI, self.e_p_integral)) - \
                          (np.dot(self.K_v, self.e_velocities)) + np.array([[0.0], [0.0], [self.mass * self.g]]) + \
                           np.multiply(self.mass, self.desired_acceleratons)

    def update_desired_values(self):
        # desired values
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

        # print "desired jerk: ", self.desired_jerk[0, 0], self.desired_jerk[1, 0], self.desired_jerk[2, 0]
        u1 = self.u[0, 0]
        dot_u1 = np.dot(self.z_B, self.desired_jerk)
        ddot_u1 = np.dot(self.z_B, self.desired_snap)
        h_omega = self.mass / self.u[0, 0] * (self.desired_jerk -
                                                (
                                                    np.multiply(
                                                        dot_u1, self.z_B.transpose()
                                                    )
                                                )
                                              )
        self.desired_omegas[0, 0] = -np.dot(self.y_B_des, h_omega)
        self.desired_omegas[1, 0] = np.dot(self.x_B_des, h_omega)
        self.desired_omegas[2, 0] = np.dot(self.z_B_des, np.array([[0.0], [0.0], [self.desired_d_yaw]]))

        h_angular_acc = self.mass / self.u[0, 0] * (self.desired_snap -
                                                    np.multiply(ddot_u1, self.z_B.T)
                                                  - np.multiply(dot_u1, np.cross(
                                                            self.angular_vel_quads.T, self.z_B).T
                                                        )
                                                  - np.cross(self.angular_vel_quads.T,
                                                             np.multiply(dot_u1, self.z_B)
                                                             + np.multiply(u1, np.cross(
                                                                     self.angular_vel_quads.T, self.z_B)
                                                               )
                                                             ).T
                                                    )

        self.desired_angular_acc[0, 0] = -np.dot(self.y_B_des, h_angular_acc)
        self.desired_angular_acc[1, 0] = np.dot(self.x_B_des, h_angular_acc)
        self.desired_angular_acc[2, 0] = np.dot(self.z_B_des, np.array([[0.0], [0.0], [self.desired_dd_yaw]]))

        vec3_omega_des = Vector3Stamped()
        vec3_omega_des.header.stamp = rospy.Time.now()
        vec3_omega_des.vector = Vector3(self.desired_omegas[0, 0], self.desired_omegas[1, 0],
                                        self.desired_omegas[2, 0])
        self.pub_desired_omegas.publish(vec3_omega_des)

    def update_desired_M(self):
        self.update_omega_err()
        self.update_R_err()
        angular_acc_tmp = \
            np.dot(
                np.dot(
                    np.dot(
                        skewsymetric(self.angular_vel_quads), self.R_.T
                    )
                , self.desired_R_
                )
            , self.desired_omegas
            )

        angular_acc_tmp = angular_acc_tmp - \
            np.dot(
                np.dot(
                    self.R_.T, self.desired_R_
                )
            , self.desired_angular_acc
            )

        self.u[1:4] = - np.multiply(self.K_R, self.e_R_) - np.multiply(self.K_omega, self.e_omegas)\
                      + np.cross(self.angular_vel_quads.T, np.dot(self.J, self.angular_vel_quads).T).T\
                      - np.dot(self.J, angular_acc_tmp)

    def motorSpeedFromU(self):
        aa = self.u
        self.motor_speed = np.dot(self.M_UtoMotor, self.u)

    def multiply_motor_speed(self, k):
        self.motor_speed = np.multiply(self.motor_speed, k)

    def load_trj_lists(self, dimension='xyz'):
        if dimension == 'xyz':
            self.trj_x_list = self.NL_planner.linear_opt.poly_pos[:, 0].tolist()
            self.trj_y_list = self.NL_planner.linear_opt.poly_pos[:, 1].tolist()
            self.trj_z_list = self.NL_planner.linear_opt.poly_pos[:, 2].tolist()

            self.trj_vx_list = self.NL_planner.linear_opt.poly_velocity[:, 0].tolist()
            self.trj_vy_list = self.NL_planner.linear_opt.poly_velocity[:, 1].tolist()
            self.trj_vz_list = self.NL_planner.linear_opt.poly_velocity[:, 2].tolist()

            self.trj_accx_list = self.NL_planner.linear_opt.poly_acc[:, 0].tolist()
            self.trj_accy_list = self.NL_planner.linear_opt.poly_acc[:, 1].tolist()
            self.trj_accz_list = self.NL_planner.linear_opt.poly_acc[:, 2].tolist()

            self.trj_jerkx_list = self.NL_planner.linear_opt.poly_acc[:, 0].tolist()
            self.trj_jerky_list = self.NL_planner.linear_opt.poly_acc[:, 1].tolist()
            self.trj_jerkz_list = self.NL_planner.linear_opt.poly_acc[:, 2].tolist()

            self.trj_snapx_list = self.NL_planner.linear_opt.poly_acc[:, 0].tolist()
            self.trj_snapy_list = self.NL_planner.linear_opt.poly_acc[:, 1].tolist()
            self.trj_snapz_list = self.NL_planner.linear_opt.poly_acc[:, 2].tolist()

        elif dimension == 'x':
            self.trj_x_list = self.NL_planner.linear_opt.poly_pos[:, 0].tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()

            self.trj_vx_list = self.NL_planner.linear_opt.poly_velocity[:, 0].tolist()
            self.trj_vy_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()
            self.trj_vz_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()

            self.trj_accx_list = self.NL_planner.linear_opt.poly_acc[:, 0].tolist()
            self.trj_accy_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 0].shape).tolist()
            self.trj_accz_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 0].shape).tolist()

            self.trj_jerkx_list = self.NL_planner.linear_opt.poly_jerk[:, 0].tolist()
            self.trj_jerky_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 0].shape).tolist()
            self.trj_jerkz_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 0].shape).tolist()

            self.trj_snapx_list = self.NL_planner.linear_opt.poly_snap[:, 0].tolist()
            self.trj_snapy_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 0].shape).tolist()
            self.trj_snapz_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 0].shape).tolist()

        elif dimension == 'y':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list = self.NL_planner.linear_opt.poly_pos[:, 1].tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()

            self.trj_vx_list = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()
            self.trj_vy_list = self.NL_planner.linear_opt.poly_velocity[:, 1].tolist()
            self.trj_vz_list = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()

            self.trj_accx_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 1].shape).tolist()
            self.trj_accy_list = self.NL_planner.linear_opt.poly_acc[:, 1].tolist()
            self.trj_accz_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 1].shape).tolist()

            self.trj_jerkx_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 1].shape).tolist()
            self.trj_jerky_list = self.NL_planner.linear_opt.poly_jerk[:, 1].tolist()
            self.trj_jerkz_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 1].shape).tolist()

            self.trj_snapx_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 1].shape).tolist()
            self.trj_snapy_list = self.NL_planner.linear_opt.poly_snap[:, 1].tolist()
            self.trj_snapz_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 1].shape).tolist()

        elif dimension == 'z':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_z_list = self.NL_planner.linear_opt.poly_pos[:, 2].tolist()

            self.trj_vx_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vy_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vz_list = self.NL_planner.linear_opt.poly_velocity[:, 2].tolist()

            self.trj_accx_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 2].shape).tolist()
            self.trj_accy_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 2].shape).tolist()
            self.trj_accz_list = self.NL_planner.linear_opt.poly_acc[:, 2].tolist()

            self.trj_jerkx_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 2].shape).tolist()
            self.trj_jerky_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 2].shape).tolist()
            self.trj_jerkz_list = self.NL_planner.linear_opt.poly_jerk[:, 2].tolist()

            self.trj_snapx_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 2].shape).tolist()
            self.trj_snapy_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 2].shape).tolist()
            self.trj_snapz_list = self.NL_planner.linear_opt.poly_snap[:, 2].tolist()

    def publish_poly3d_point(self, xyz_arr, v_xyz_arr, acc_xyz_arr, jerk_xyz_arr, snap_xyz_arr):
        self.desired_positions = xyz_arr.reshape((3, 1))
        self.desired_velocities = v_xyz_arr.reshape((3, 1))
        self.desired_acceleratons = acc_xyz_arr.reshape((3, 1))
        self.desired_jerk = jerk_xyz_arr.reshape((3, 1))
        self.desired_snap = snap_xyz_arr.reshape((3, 1))
        self.publish_desired_trj()

    def publish_poly3d_trj(self):
        if self.isPolynomialSolved:
            if len(self.trj_x_list) == 1:
                trj_tmp = np.array([self.trj_x_list[0], self.trj_y_list[0], self.trj_z_list[0]])
                trj_v_tmp = np.array([self.trj_vx_list[0], self.trj_vy_list[0], self.trj_vz_list[0]])
                trj_acc_tmp = np.array([self.trj_accx_list[0], self.trj_accy_list[0], self.trj_accz_list[0]])
                trj_jerk_tmp = np.array([self.trj_jerkx_list[0], self.trj_jerky_list[0], self.trj_jerkz_list[0]])
                trj_snap_tmp = np.array([self.trj_snapx_list[0], self.trj_snapy_list[0], self.trj_snapz_list[0]])
            else:
                trj_tmp = np.array([self.trj_x_list.pop(0), self.trj_y_list.pop(0), self.trj_z_list.pop(0)])
                trj_v_tmp = np.array([self.trj_vx_list.pop(0), self.trj_vy_list.pop(0), self.trj_vz_list.pop(0)])
                trj_acc_tmp = np.array([self.trj_accx_list.pop(0), self.trj_accy_list.pop(0), self.trj_accz_list.pop(0)])
                trj_jerk_tmp = np.array([self.trj_jerkx_list.pop(0), self.trj_jerky_list.pop(0), self.trj_jerkz_list.pop(0)])
                trj_snap_tmp = np.array([self.trj_snapx_list.pop(0), self.trj_snapy_list.pop(0), self.trj_snapz_list.pop(0)])
            self.publish_poly3d_point(trj_tmp, trj_v_tmp, trj_acc_tmp, trj_jerk_tmp, trj_snap_tmp)
        else:
            print "Polynomial has not been solved yet! Please solve the poly coefficients first!"

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
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def run(self):
        self.optimize()
        self.getPlanUpToSnap(frequency=50)
        self.load_trj_lists()

        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def hover_and_trj_xy(self, dimension='x'):
        self.optimize()
        self.getPlanUpToSnap(frequency=50)

        # self.calculatePosVelAcc()
        # self.NL_planner.linear_opt.plot_derivatives(order=0)
        # self.NL_planner.linear_opt.plot_derivatives(order=1)
        # self.NL_planner.linear_opt.plot_derivatives(order=2)
        # self.NL_planner.linear_opt.plot_derivatives(order=3)
        # self.NL_planner.linear_opt.plot_derivatives(order=4)

        '''
        init ROS node
        '''
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)

        '''
        ROS Loop
        '''
        while not rospy.is_shutdown():
            if self.hover_duration < 5.0:
                self.set_hover_des(target_height=1.5)
            else:
                if not self.offset_added:
                    print "hovering finished, going into the next phase..."
                    self.update_offset()
                    self.load_trj_lists(dimension=dimension)
                    print "offset added"
                self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()


if __name__ == '__main__':
    dimension = 3
    mellinger_nl = Mellinger('hummingbird', 0.0, 0.0, 0.0, dimension=dimension, N=10)

    mellinger_nl.NL_planner.setVerticesPosVel(
                               positions=[
                                        [0.0, 0.0, 0.0],
                                        [2.0, 0.0, 2.0],
                                        [4.0, 0.0, 5.0]
                               ], velocities=[
                                 [0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0]
                                  ])
    mellinger_nl.hover_and_trj_xy()





