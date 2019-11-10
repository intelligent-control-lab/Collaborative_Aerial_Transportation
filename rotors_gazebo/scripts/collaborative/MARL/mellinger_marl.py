import numpy as np
from rotors_gazebo.msg import Float64ArrayStamped
from payload_distributor_marl import *
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3Stamped, Vector3
from nav_msgs.msg import Odometry

import sys
sys.path.insert(0, '..')
sys.path.append("/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative")
from mellinger_trj_nlopt import *


class Mellinger_Agent(Mellinger):
    def __init__(self, mav_name, index, num_agents, c, x, y, z, dim=3, N=10):
        Mellinger.__init__(self, mav_name, index, x, y, z, dimension=dim, N=N)

        self.num_agents = num_agents
        # who-to-blame related parameters
        self.relative_positions = [np.zeros((3, 1)) for _ in range(self.num_agents)]
        self.relative_positions[0][0, 0] = 0.95
        self.relative_positions[0][1, 0] = 0.0
        self.relative_positions[0][2, 0] = 0.35 - self.payload_center[2, 0]

        self.relative_positions[1][0, 0] = 0.0
        self.relative_positions[1][1, 0] = 0.95
        self.relative_positions[1][2, 0] = 0.35 - self.payload_center[2, 0]

        self.relative_positions[2][0, 0] = -0.95
        self.relative_positions[2][1, 0] = 0.0
        self.relative_positions[2][2, 0] = 0.35 - self.payload_center[2, 0]

        self.relative_positions[3][0, 0] = 0.0
        self.relative_positions[3][1, 0] = -0.95
        self.relative_positions[3][2, 0] = 0.35 - self.payload_center[2, 0]

        self.relative_pos = np.array([[x], [y], [z]]) - self.payload_center
        c_init = [0.4, 0.1, 0.15, 0.35]

        self.sum_force = 0.0
        self.c = c
        self.estimated_c = c_init
        self.estimated_c[index] = c

        # rand = np.random.rand(1, 3)
        # count = 0
        # for i in range(self.num_agents):
        #     if i != index:
        #         self.estimated_c[i] = (rand[0, count] / np.sum(rand)) * (1-self.c)
        #         count = count + 1

        self.running_frequency = 50.0
        self.dt = 1.0 / self.running_frequency

        self.distributor = PayloadDistributor(c_init, index)
        self.c_hat = [0.0 for _ in range(4)]
        self.c_E_ = np.zeros((6, 6))
        self.M_ = np.zeros((6, 6))
        self.M_[0:3, 0:3] = self.mass * np.eye(3)
        self.M_[3:, 3:] = self.J
        self.W_ = np.zeros((6, 1))
        self.W_[2, 0] = self.g
        self.concateAccAngularacc = np.zeros((6, 1))
        self.D_ = np.zeros((6, 6))
        self.G_ = np.zeros((6, 6))
        self.c_M0 = np.zeros((6, 6))
        self.c_W0 = np.zeros((6, 6))
        self.index = index
        self.update_distributor_matrices()

        # ROS topic to publish the current estimation of the c parameters
        self.topic_c = self.name + '/' + 'estimation_c'
        self.pub_c = rospy.Publisher(self.topic_c, Float64ArrayStamped, queue_size=1)

        topic_tf = self.name + '/' + 'f_t'
        self.sub_force_torque = rospy.Subscriber(topic_tf, WrenchStamped, self.callback_ft)
        self.sensor_force = np.zeros((3, 1))
        self.sensor_torque = np.zeros((3, 1))
        self.reward_ft = 0.0
        self.reward_payload = 0.0
        self.reward_arrive = 0.0

        topic_name_desired_wrench = '/' + self.name + '/commander/ref_ft'
        self.pub_desired_wrench = rospy.Publisher(topic_name_desired_wrench, WrenchStamped, queue_size=1)
        self.ref_ft = WrenchStamped()

        topic_name_err_wrench = '/' + self.name + '/commander/err_wrench'
        self.pub_err_wrench = rospy.Publisher(topic_name_err_wrench, WrenchStamped, queue_size=1)
        self.err_ft = WrenchStamped()

    def update_distributor_matrices(self):
        print "self.estimated_c: ", self.estimated_c
        self.distributor.update_c(self.estimated_c)
        self.distributor.update_cE()
        self.c_E_ = self.distributor.c_E
        self.distributor.updateM0W0()

        self.c_M0 = np.multiply(self.c, self.get_M0())
        self.c_W0 = np.multiply(self.c, self.get_W0())

        
        self.D_ = self.M_ + self.c_M0
        self.G_ = self.W_ + self.c_W0

    def get_M0(self):
        return self.distributor.getM0()

    def get_W0(self):
        return self.distributor.getW0()

    def get_c(self):
        return self.estimated_c

    def publish_c(self):
        float64arr = Float64ArrayStamped()
        float64arr.header.stamp = rospy.Time.now()
        float64arr.data = self.estimated_c
        self.pub_c.publish(float64arr)

    def update_controller(self):
        self.concateAccAngularacc = np.concatenate(
            (self.desired_acceleratons, self.desired_angular_acc), axis=0
        )

    def reward_arrive_goal(self, goal_position, epsilon=0.2):
        err_to_goal = self.payload_position - np.array(goal_position).reshape((3, 1))
        assert(err_to_goal.shape == (3, 1))
        abs_dist = np.linalg.norm(err_to_goal)
        print "does it arrive? abs_dist:  ", abs_dist
        if abs_dist < epsilon:
            self.reward_arrive = 500.0
        else:
            self.reward_arrive = 0.0

    def update_estimated_F(self):
        return

    def callback_ft(self, data):
        self.sensor_force = np.array([[data.wrench.force.x], [data.wrench.force.y], [data.wrench.force.z]]) 
        self.sensor_torque = np.array([[data.wrench.torque.x], [data.wrench.torque.y], [data.wrench.torque.z]])

        self.sum_force = self.sum_force + data.wrench.force.x
        # print "sensor_force: ", self.sensor_force[0, 0], self.sensor_force[1, 0], self.sensor_force[2, 0]
        # print "sensor_torque: ", self.sensor_torque[0, 0], self.sensor_torque[1, 0], self.sensor_torque[2, 0]

    def state_err_payload(self):
        payload_pos_err = self.payload_position - self.des_payload_position
        return np.linalg.norm(payload_pos_err)

    def WrenchErr(self):
        force_net_err = np.multiply(self.mass, self.c_W0[0:3]) - self.sensor_force
        # print "torque err: ", np.linalg.norm(force_net_err)
        return np.linalg.norm(force_net_err)

    def update_desired_F_Distributor(self):
        self.update_desired_F()
        # print "self.desired_F before distributor: ", self.desired_F
        # hzyu
        self.desired_F = self.desired_F \
                         + np.dot(self.c_M0[0:3, :], self.concateAccAngularacc)
        self.desired_F = self.desired_F + np.multiply(self.mass, self.c_W0[0:3])

        # self.ref_ft.wrench.force = Vector3(self.desired_F[0], self.desired_F[1], self.desired_F[2])

    def update_desired_M_Distributor(self):
        self.update_desired_M()
        # hzyu
        self.desired_M = self.desired_M + np.dot(self.c_M0[3:, :], self.concateAccAngularacc) + self.c_W0[3:]
        self.u[1:4] = self.desired_M

        # self.ref_ft.wrench.torque = Vector3(self.desired_M[0], self.desired_M[1], self.desired_M[2])
        # self.ref_ft.header.stamp = rospy.Time.now()
        # self.pub_desired_wrench.publish(self.ref_ft)

        # self.reward_ft = self.WrenchErr()
        self.reward_payload = self.state_err_payload()
        # print "self.reward_payload: ", self.reward_payload

        # err_f = np.multiply(self.mass, self.c_W0[0:3]) - self.sensor_force 
        # err_t = self.desired_M - self.sensor_torque
        # self.err_ft.header.stamp = rospy.Time.now()
        # self.err_ft.wrench.force = Vector3(err_f[0], err_f[1], err_f[2])
        # self.err_ft.wrench.torque = Vector3(err_t[0], err_t[1], err_t[2])
        # self.pub_err_wrench.publish(self.err_ft)

        # self.u[1:4] = self.u[1:4] + np.dot(self.c_M0[3:, :], self.concateAccAngularacc)
        # self.u[1:4] = self.u[1:4] + self.c_W0[3:]

    def update_estimated_c(self, c):
        count = 0
        for i in range(self.num_agents):
            if i == self.index:
                continue
            self.estimated_c[i] = c[count]
            count = count + 1
        # print "update_estimated_c: ", self.estimated_c

    def update_des_distributor(self):
        self.update_distributor_matrices()
        self.update_controller()
        self.update_desired_F_Distributor()
        self.update_desired_values()
        self.update_desired_M_Distributor()

    def hover_and_trj_xy(self, dimension='x'):
        self.optimize()
        self.getPlanUpToSnap(frequency=self.frequency)
        '''
        init ROS node
        '''
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        # rospy.sleep(0.5)
        '''
        ROS Loop
        '''
        while not rospy.is_shutdown():
            if self.hover_duration < 4.0:
                self.set_hover_des(target_height=1.5)
            else:
                if not self.offset_added:
                    print "hovering finished, going into the next phase..."
                    print "before:  ", self.NL_planner.linear_opt.poly_pos[0, 0], ' ',self.NL_planner.linear_opt.poly_pos[0, 1], ' ',self.NL_planner.linear_opt.poly_pos[0, 2]
                    self.update_offset_xyz(self.positions_quads[0, 0], self.positions_quads[1, 0], self.positions_quads[2, 0])
                    print "after:  ", self.NL_planner.linear_opt.poly_pos[0, 0], ' ',self.NL_planner.linear_opt.poly_pos[0, 1], ' ',self.NL_planner.linear_opt.poly_pos[0, 2]
                    self.load_trj_lists(dimension=dimension)
                    print "offset added"
                self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_des_distributor()
            self.motorSpeedFromU()
            self.send_motor_command()
            # self.rate.sleep()


class CentralizedControllerDistributor(object):
    def __init__(self, name='controller'):
        dimension = 3
        self.name = name
        self.controllers = [
            Mellinger_Agent(
                mav_name='hummingbird',
                index=0,
                num_agents=4,
                c=0.4,
                x=0.95, y=0.0, z=0.25,
                dim=dimension
            ),
            Mellinger_Agent(
                'hummingbird',
                index=1,
                num_agents=4,
                c=0.1,
                x=0.0, y=0.95, z=0.25,
                dim=dimension
            ),
            Mellinger_Agent(
                'hummingbird',
                index=2,
                num_agents=4,
                c=0.25,
                x=-0.95, y=0.0, z=0.25,
                dim=dimension
            ),
            Mellinger_Agent(
                'hummingbird',
                index=3,
                num_agents=4,
                c=0.25,
                x=0.0, y=-0.95, z=0.25,
                dim=dimension
            )
        ]
        self.num_of_quads = 4

        self.initial_time = None
        self.rate = None
        self.t = None
        self.hover_begin_time = None
        self.hover_duration = 0.0
        self.frequency = 50.0

        positions = [[0.0, 0.0, 0.0],
                     [2.0, 0.0, 2.0],
                     [4.0, 0.0, 4.0]]
        velocities = [[0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0]]

        for controller in self.controllers:
            controller.NL_planner.setVerticesPosVel(positions, velocities)

    def hover_and_trj_xy(self, dimension='xyz'):
        for nl_opt in self.controllers:
            nl_opt.optimize()
            nl_opt.getPlanUpToSnap(frequency=self.frequency)

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
            for i_mellinger in self.controllers:
                if i_mellinger.hover_duration < 3.0:
                    i_mellinger.set_hover_des(target_height=1.5)
                else:
                    if not i_mellinger.offset_added:
                        print "hovering finished, going into the next phase..."
                        i_mellinger.update_offset_xyz(i_mellinger.positions_quads[0, 0], i_mellinger.positions_quads[1, 0], i_mellinger.positions_quads[2, 0])
                        i_mellinger.load_trj_lists(dimension=dimension)
                        print "offset added"
                    i_mellinger.publish_poly3d_trj()
                i_mellinger.publish_err()
                i_mellinger.update_current_state()
                i_mellinger.update_des_distributor()
                i_mellinger.motorSpeedFromU()
                i_mellinger.send_motor_command()
            self.rate.sleep()


if __name__ == '__main__':
    central_controller_distributor = CentralizedControllerDistributor()
    central_controller_distributor.hover_and_trj_xy()






