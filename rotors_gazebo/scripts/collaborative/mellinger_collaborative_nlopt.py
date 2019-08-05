from mellinger_trj_nlopt import Mellinger
import rospy
from polynomialTrjNonlinear.vertex import Vertex


class CentrolizedController(object):
    def __init__(self, name='controller'):
        dimension = 3
        self.name = name
        self.controllers = [
                      Mellinger('hummingbird_0', 0.95, 0.0, 0.35),
                      Mellinger('hummingbird_1', 0.0, 0.95, 0.35),
                      Mellinger('hummingbird_2', -0.95, 0.0, 0.35),
                      Mellinger('hummingbird_3', 0.0, -0.95, 0.35)
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

        # vertices = []
        # vertex0 = Vertex(dimension=dimension, index=0)
        # start_position = [0.0, 0.0, 0.0]
        # start_velocity = [0.0, 0.0, 0.0]
        # for nl_opt in self.controllers:
        #     nl_opt.NL_planner.construct_constrain(vertex0, order=0, input_list=start_position)
        #     nl_opt.NL_planner.construct_constrain(vertex0, order=1, input_list=start_velocity)
        #
        # vertex0.makeStartOrEnd(position=start_position, up_to_order=4)
        #
        # vertex1 = Vertex(dimension=dimension, index=1)
        #
        # position = [2.0, 0.0, 2.0]
        # velocity = [0.0, 0.0, 0.0]
        # for nl_opt in self.controllers:
        #     nl_opt.NL_planner.construct_constrain(vertex1, order=0, input_list=position)
        #     nl_opt.NL_planner.construct_constrain(vertex1, order=1, input_list=velocity)
        #
        # vertex2 = Vertex(dimension=dimension, index=2)
        # position = [4.0, 0.0, 4.0]
        # velocity = [0.0, 0.0, 0.0]
        # for nl_opt in self.controllers:
        #     nl_opt.NL_planner.construct_constrain(vertex2, order=0, input_list=position)
        #     nl_opt.NL_planner.construct_constrain(vertex2, order=1, input_list=velocity)
        #
        # vertex2.makeStartOrEnd(position=position, up_to_order=4)
        #
        # vertices.append(vertex0)
        # vertices.append(vertex1)
        # vertices.append(vertex2)
        #
        # for nl_opt in self.controllers:
        #     nl_opt.NL_planner.setupFromVertices(vertices)
        #     nl_opt.NL_planner.add_max_vel(2.0)
        #     nl_opt.NL_planner.add_max_acc(1.0)

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
                i_mellinger.update_desired_inputs()
                i_mellinger.motorSpeedFromU()
                i_mellinger.send_motor_command()
            self.rate.sleep()


if __name__ == '__main__':
    central_controller = CentrolizedController()
    central_controller.hover_and_trj_xy()

