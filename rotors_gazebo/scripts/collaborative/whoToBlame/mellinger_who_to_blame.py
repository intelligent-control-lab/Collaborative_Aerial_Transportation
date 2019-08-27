import numpy as np
from mellinger_trj_nlopt import *
from payload_distributor_whotoblame_ros import *
from rotors_gazebo.msg import Float64ArrayStamped


class MellingerWhoToBlame(Mellinger):
    def __init__(self, mav_name, index, num_agents, c, x, y, z, dim=3, N=10, strategy='blame_all'):
        Mellinger.__init__(self, mav_name, index, x, y, z, dimension=dim, N=N)

        self.num_agents = num_agents
        # who-to-blame related parameters
        self.relative_positions = [np.zeros((3, 1)) for _ in range(self.num_agents)]
        self.relative_positions[0][0, 0] = 0.95
        self.relative_positions[0][1, 0] = 0.0
        self.relative_positions[0][2, 0] = 0.35

        self.relative_positions[0][0, 0] = 0.0
        self.relative_positions[0][1, 0] = 0.95
        self.relative_positions[0][2, 0] = 0.35

        self.relative_positions[0][0, 0] = -0.95
        self.relative_positions[0][1, 0] = 0.0
        self.relative_positions[0][2, 0] = 0.35

        self.relative_positions[0][0, 0] = 0.0
        self.relative_positions[0][1, 0] = -0.95
        self.relative_positions[0][2, 0] = 0.35

        c_init = list(np.ones((self.num_agents,)) / self.num_agents)

        self.c = c
        self.estimated_c = c_init
        self.strategy = strategy
        self.estimated_inputs = [np.zeros((3, 1)) for _ in range(self.num_agents)]
        # [velocities, angular_velocities]
        self.estimated_states = [np.zeros((6, 1)) for _ in range(self.num_agents)]
        self.estimated_forces = [np.zeros((3, 1)) for _ in range(self.num_agents)]
        self.estimated_torques = [np.zeros((3, 1)) for _ in range(self.num_agents)]

        self.running_frequency = 50.0
        self.dt = 1.0 / self.running_frequency

        self.strategy = strategy
        if self.strategy == 'blame_me':
            c_init[index] = c
        self.distributor = PayloadDistributor(mav_name, c_init, index)
        self.c_hat = [0.0 for _ in range(4)]
        self.c_E_ = np.zeros((6, 6))
        self.M_ = np.zeros((6, 6))
        self.M_[0:3, 0:3] = self.mass * np.eye(3)
        self.M_[3:, 3:] = self.J
        self.W_ = np.zeros((6, 1))
        self.W_[2, 0] = self.g
        self.concateAccAngularacc = np.zeros((6, 1))
        self.D_ = [np.zeros((6, 6)) for _ in range(self.num_agents)]
        self.G_ = [np.zeros((6, 6)) for _ in range(self.num_agents)]
        self.c_M0 = [np.zeros((6, 6)) for _ in range(self.num_agents)]
        self.c_W0 = [np.zeros((6, 6)) for _ in range(self.num_agents)]
        self.index = index
        self.update_distributor_matrices()

        # ROS topic to publish the current estimation of the c parameters
        self.topic_c = self.name + '/' + 'estimation_c'
        self.pub_c = rospy.Publisher(self.topic_c, Float64ArrayStamped, queue_size=10)

    def set_strategy(self, strategy):
        if strategy in ['blame_me', 'blame_all']:
            self.strategy = strategy
        else:
            print "invalid strategy name, should be blame_me or blame_all..."

    def update_distributor_matrices(self):
        self.distributor.update_c(self.estimated_c)
        self.distributor.update_cE()
        self.c_E_ = self.distributor.c_E
        self.distributor.updateM0W0()

        for i_agent in range(self.num_agents):
            self.c_M0[i_agent] = self.c * self.get_M0(i_agent)
            self.c_W0[i_agent] = self.c * self.get_W0(i_agent)
            self.D_[i_agent] = self.M_[i_agent] + self.c_M0[i_agent]
            self.G_[i_agent] = self.W_[i_agent] + self.c_W0[i_agent]

    def get_M0(self, index):
        return self.distributor.getM0(index)

    def get_W0(self, index):
        return self.distributor.getW0(index)

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

    def update_estimated_F(self):
        return

    def update_desired_F_Distributor(self):
        self.update_desired_F()
        self.desired_F = self.desired_F \
                         + np.dot(self.c_M0[0:3, :], self.concateAccAngularacc)
        self.desired_F = self.desired_F + np.multiply(self.mass, self.c_W0[0:3])

    def update_desired_M_Distributor(self):
        self.update_desired_M()
        self.u[1:4] = self.u[1:4] + np.dot(self.c_M0[3:, :], self.concateAccAngularacc)
        self.u[1:4] = self.u[1:4] + self.c_W0[3:]

    def update_estimated_states(self):
        return

    def update_estimated_c(self):
        return

    def update_des_distributor(self):
        self.update_distributor_matrices()
        self.update_controller()
        self.update_desired_F_Distributor()
        self.update_desired_values()
        self.update_desired_M_Distributor()

    def hover_and_trj_xy(self, dimension='x'):
        self.optimize()
        self.getPlanUpToSnap(frequency=50)
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
            self.update_des_distributor()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()


class CentralizedControllerDistributor(object):
    def __init__(self, name='controller'):
        dimension = 3
        self.name = name
        self.controllers = [
            MellingerWhoToBlame(
                mav_name='hummingbird',
                index=0,
                num_agents=4,
                c=0.45,
                x=0.95, y=0.0, z=0.35,
                dim=dimension
            ),
            MellingerWhoToBlame(
                'hummingbird',
                index=1,
                num_agents=4,
                c=0.05,
                x=0.0, y=0.95, z=0.35,
                dim=dimension
            ),
            MellingerWhoToBlame(
                'hummingbird',
                index=2,
                num_agents=4,
                c=0.15,
                x=-0.95, y=0.0, z=0.35,
                dim=dimension
            ),
            MellingerWhoToBlame(
                'hummingbird',
                index=3,
                num_agents=4,
                c=0.35,
                x=0.0, y=-0.95, z=0.35,
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
            nl_opt.getPlanUpToSnap(frequency=50)

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
                if i_mellinger.hover_duration < 5.0:
                    i_mellinger.set_hover_des(target_height=1.5)
                else:
                    if not i_mellinger.offset_added:
                        print "hovering finished, going into the next phase..."
                        i_mellinger.update_offset()
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






