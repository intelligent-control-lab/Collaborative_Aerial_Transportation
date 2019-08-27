from consensus.mellinger_consensus import *
from payload_consensus_distributor_ros import *


class CentralizedControllerDistributor(object):
    def __init__(self, name='controller'):
        self.name = name
        dict_neighbours = {0: [1, 3],
                           1: [2, 0],
                           2: [3, 1],
                           3: [0, 2]}
        self.controller = MellingerWithDistributor(
                                                mav_name='hummingbird',
                                                num_agents=4,
                                                dict_neighbours=dict_neighbours,
                                                c_init=[0.45, 0.05, 0.15, 0.35],
                                                x=[0.95, 0.0, -0.95, 0.0],
                                                y=[0.0, 0.95, 0.0, -0.95],
                                                z=[0.35, 0.35, 0.35, 0.35],
                                                dim=3
                                                )
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

        for i_mellinger in self.controller.Mellingers:
            i_mellinger.NL_planner.setVerticesPosVel(positions, velocities)

    def hover_and_trj_xy(self, dimension='xyz'):
        for nl_opt in self.controller.Mellingers:
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
        for i, i_mellinger in enumerate(self.controller.Mellingers):
            i_mellinger.update_current_state()
        while not rospy.is_shutdown():
            self.controller.update_des_distributor()
            for i_mellinger in self.controller.Mellingers:
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
            self.rate.sleep()


if __name__ == '__main__':
    central_controller_distributor = CentralizedControllerDistributor()
    central_controller_distributor.hover_and_trj_xy()






