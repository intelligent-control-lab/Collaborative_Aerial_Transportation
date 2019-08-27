from mellinger_trj_nlopt import Mellinger
import rospy
from rotors_gazebo.msg import Float64Stamped
from payload_consensus_distributor_ros import *


class MellingerWithNeighbours(Mellinger):
    """
    Mellinger Controller with Polynomial Trajectory Generation
    Using nonlinear optimizations.
    """

    def __init__(self, mav_name, index, list_neighbour_indx, c, x, y, z, dimension=3, N=10):
        Mellinger.__init__(self, mav_name, index, x, y, z, dimension, N)
        self.c = c
        self.neighbours = list_neighbour_indx
        self.num_neighbours = len(list_neighbour_indx)

        # subscriber for the c_j of its neighbours:
        self.sub_neighbours = [None for _ in range(self.num_neighbours)]
        self.cb_neighbours = [self.cb_neighbour_0]
        self.cb_neighbours.append(self.cb_neighbour_1)
        # self.cb_neighbours.append(self.cb_neighbour_2) ...
        self.c_js = [0.0 for _ in range(self.num_neighbours)]
        self.lbd = 0.25
        for j, j_neighbour in enumerate(self.neighbours):
            topic_name_cj = '/' + mav_name + '_' + str(j_neighbour) + '/' + 'c'
            self.sub_neighbours[j] = rospy.Subscriber(topic_name_cj, Float64Stamped, self.cb_neighbours[j])

        # publisher for the distribution:
        topic_name_c = '/' + self.name + '/' + 'c'
        self.pub_c = rospy.Publisher(topic_name_c, Float64Stamped, queue_size=2)

    def update_c(self):
        c_modify = 0.0
        for c_j in self.c_js:
            c_modify = self.lbd * (c_j - self.c)
        self.c = self.c + c_modify
        self.publish_c()

    def publish_c(self):
        float_c = Float64Stamped()
        float_c.header.stamp = rospy.Time.now()
        float_c.data = self.c
        self.pub_c.publish(float_c)

    # callback
    def cb_neighbour_0(self, data):
        c_j = data.data
        self.c_js[0] = c_j

    def cb_neighbour_1(self, data):
        c_j = data.data
        self.c_js[1] = c_j


class MellingerWithDistributor(PayloadDistributor):
    def __init__(self, mav_name, num_agents, dict_neighbours, c_init, x, y, z, dim=3):
        PayloadDistributor.__init__(self, mav_name, c_init)
        self.Mellingers = [MellingerWithNeighbours(mav_name, i, dict_neighbours[i], c_init[i], x[i], y[i], z[i], dimension=dim, N=10)
                           for i in range(num_agents)]

        self.num_agents = num_agents
        self.c_E_ = [np.zeros((6, 6)) for _ in range(num_agents)]
        self.M_ = [np.zeros((6, 6)) for _ in range(num_agents)]
        for idx, i_mellinger in enumerate(self.Mellingers):
            self.M_[idx][0:3, 0:3] = i_mellinger.mass * np.eye(3)
            self.M_[idx][3:, 3:] = i_mellinger.J
        self.W_ = [np.zeros((6, 1)) for _ in range(num_agents)]
        for idx, i_mellinger in enumerate(self.Mellingers):
            self.W_[idx][2, 0] = i_mellinger.g
        self.concateAccAngularacc = [np.zeros((6, 1)) for _ in range(num_agents)]
        self.D_ = [np.zeros((6, 6)) for _ in range(num_agents)]
        self.G_ = [np.zeros((6, 6)) for _ in range(num_agents)]
        self.c_M0 = [np.zeros((6, 6)) for _ in range(num_agents)]
        self.c_W0 = [np.zeros((6, 6)) for _ in range(num_agents)]
        self.update_distributor_matrices()

    def update_distributor_matrices(self):
        self.update_cE()
        self.c_E_ = self.c_E
        self.updateM0W0()
        for indx in range(self.num_agents):
            self.c_M0[indx] = self.c[indx] * self.getM0(indx)
            self.c_W0[indx] = self.c[indx] * self.getW0(indx)
            self.D_[indx] = self.M_[indx] + self.c_M0[indx]
            self.G_[indx] = self.W_[indx] + self.c_W0[indx]

    def get_c_by_index(self, index):
        return self.get_c(index)

    def update_controller(self):
        for i, i_mellinger in enumerate(self.Mellingers):
            i_mellinger.desired_angular_acc = np.zeros((3, 1))
            self.concateAccAngularacc[i] = np.concatenate(
                (i_mellinger.desired_acceleratons, i_mellinger.desired_angular_acc), axis=0
            )

    def update_desired_F_Distributor(self):
        for i, i_mellinger in enumerate(self.Mellingers):
            i_mellinger.update_desired_F()
            i_mellinger.desired_F = i_mellinger.desired_F \
                         + np.dot(self.c_M0[i][0:3, :], self.concateAccAngularacc[i])
            i_mellinger.desired_F = i_mellinger.desired_F + np.multiply(i_mellinger.mass, self.c_W0[i][0:3])

    def update_desired_M_Distributor(self):
        for i, i_mellinger in enumerate(self.Mellingers):
            i_mellinger.update_desired_M()
            i_mellinger.u[1:4] = i_mellinger.u[1:4] + np.dot(self.c_M0[i][3:, :], self.concateAccAngularacc[i])
            i_mellinger.u[1:4] = i_mellinger.u[1:4] + self.c_W0[i][3:]

    def update_desired_values(self):
        for i, i_mellinger in enumerate(self.Mellingers):
            i_mellinger.update_desired_values()

    def update_des_distributor(self):
        self.update_distributor_matrices()
        self.update_controller()
        self.update_desired_F_Distributor()
        self.update_desired_values()
        self.update_desired_M_Distributor()

    def hover_and_trj_xy(self, dimension='x'):
        for i, i_mellinger in enumerate(self.Mellingers):
            i_mellinger.optimize()
            i_mellinger.getPlanUpToSnap(frequency=50)
            '''
            init ROS node
            '''
            rospy.init_node(i_mellinger.name, anonymous=True)
            # Timer
            i_mellinger.initial_time = rospy.Time.now()
            i_mellinger.t = i_mellinger.initial_time
            i_mellinger.rate = rospy.Rate(i_mellinger.frequency)
            rospy.sleep(3.0)
        '''
        ROS Loop
        '''
        while not rospy.is_shutdown():
            self.update_des_distributor()
            for i, i_mellinger in enumerate(self.Mellingers):
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
                i_mellinger.motorSpeedFromU()
                i_mellinger.send_motor_command()
                i_mellinger.rate.sleep()


if __name__ == '__main__':
    dimension = 3
    mellinger_nl = MellingerWithNeighbours('hummingbird', 0.0, 0.0, 0.0, dimension=dimension, N=10)

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





