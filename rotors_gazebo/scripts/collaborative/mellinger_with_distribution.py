from mellinger_trj_nlopt import *
from payload_distributor import *


class MellingerWithDistributor(Mellinger):
    def __init__(self, mav_name, index, x, y, z, dim=3):
        Mellinger.__init__(self, mav_name, index, x, y, z, dimension=dim, N=10)
        c_init = list(np.ones((4,1)) / 4)
        self.distributor = PayloadDistributor(c_init, index)
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
        self.update_distributor()

    def update_distributor(self):
        self.distributor.update_cE()
        self.c_E_ = self.distributor.c_E
        self.distributor.updateM0W0()

        self.c_M0 = self.get_c() * self.get_M0()
        self.c_W0 = self.get_c() * self.get_W0()
        self.D_ = self.M_ + self.c_M0
        self.G_ = self.W_ + self.c_W0

    def get_M0(self):
        return self.distributor.getM0()

    def get_W0(self):
        return self.distributor.getW0()

    def get_c(self):
        print "c: ", self.distributor.get_c(self.index)
        return self.distributor.get_c(self.index)

    def update_controller(self):
        self.concateAccAngularacc = np.concatenate(
            (self.desired_acceleratons, self.desired_angular_acc), axis=0
        )

    def update_desired_F_Distributor(self):
        self.update_desired_F()
        self.desired_F = self.desired_F \
                         + np.dot(self.c_M0[0:3, :], self.concateAccAngularacc)
        self.desired_F = self.desired_F + np.multiply(self.mass, self.c_W0[0:3])

    def update_desired_M_Distributor(self):
        self.update_desired_M()
        self.u[1:4] = self.u[1:4] + np.dot(self.c_M0[3:, :], self.concateAccAngularacc)
        self.u[1:4] = self.u[1:4] + self.c_W0[3:]

    def update_des_distributor(self):
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
                      MellingerWithDistributor(
                          mav_name='hummingbird',
                          index=0,
                          x=0.95, y=0.0, z=0.25,
                          dim=dimension
                      ),
                      MellingerWithDistributor(
                          'hummingbird',
                          index=1,
                          x=0.0, y=0.95, z=0.25,
                          dim=dimension
                      ),
                      MellingerWithDistributor(
                          'hummingbird',
                          index=2,
                          x=-0.95, y=0.0, z=0.25,
                          dim=dimension
                      ),
                      MellingerWithDistributor(
                          'hummingbird',
                          index=3,
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
            nl_opt.getPlanUpToSnap(frequency=50.0)

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
                if i_mellinger.hover_duration < 1.5:
                    i_mellinger.set_hover_des(target_height=1.5)
                else:
                    i_mellinger.update_offset_xyz(i_mellinger.payload_position[0, 0], i_mellinger.payload_position[1, 0], i_mellinger.payload_position[2, 0])
                    i_mellinger.load_ref_trj_payload(dimension=dimension)
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
    '''
    init ROS node
    '''
    rospy.init_node('controller', anonymous=True)
    central_controller_distributor = CentralizedControllerDistributor()
    central_controller_distributor.hover_and_trj_xy()






