from mellinger_trj_nlopt import Mellinger
import rospy
from polynomialTrjNonlinear.vertex import Vertex


class CentrolizedController(object):
    def __init__(self):
        dimension = 3
        self.quads = [
                      Mellinger('hummingbird_0', 0.95, 0.0, 0.35),
                      Mellinger('hummingbird_1', 0.0, 0.95, 0.35),
                      Mellinger('hummingbird_2', -0.95, 0.0, 0.35),
                      Mellinger('hummingbird_3', 0.0, -0.95, 0.35)
                      ]
        self.num_of_quads = 4

        vertices = []
        vertex0 = Vertex(dimension=dimension, index=0)
        start_position = [0.0, 0.0, 0.0]
        start_velocity = [0.0, 0.0, 0.0]
        for nl_opt in self.quads:
            nl_opt.planner.construct_constrain(vertex0, order=0, input_list=start_position)
            nl_opt.planner.construct_constrain(vertex0, order=1, input_list=start_velocity)

        vertex0.makeStartOrEnd(position=start_position, up_to_order=4)

        vertex1 = Vertex(dimension=dimension, index=1)

        position = [2.0, 0.0, 2.0]
        velocity = [0.0, 0.0, 0.0]
        for nl_opt in self.quads:
            nl_opt.planner.construct_constrain(vertex1, order=0, input_list=position)
            nl_opt.planner.construct_constrain(vertex1, order=1, input_list=velocity)

        vertex2 = Vertex(dimension=dimension, index=2)
        position = [4.0, 0.0, 4.0]
        velocity = [0.0, 0.0, 0.0]
        for nl_opt in self.quads:
            nl_opt.planner.construct_constrain(vertex2, order=0, input_list=position)
            nl_opt.planner.construct_constrain(vertex2, order=1, input_list=velocity)

        vertex2.makeStartOrEnd(position=position, up_to_order=4)

        vertices.append(vertex0)
        vertices.append(vertex1)
        vertices.append(vertex2)

        times = [3.89891, 3.91507]

        for nl_opt in self.quads:
            nl_opt.planner.setupFromVertices(vertices, times)
            nl_opt.planner.add_max_vel(2.0)
            nl_opt.planner.add_max_acc(1.0)


if __name__ == '__main__':
    central_controller = CentrolizedController()
    for i_controller in central_controller.quads:
        i_controller.optimize()
        i_controller.get_planned_pos_vel()
        i_controller.load_trj_lists()
    rospy.init_node("controller", anonymous=True)
    rate = rospy.Rate(50)
    rospy.sleep(3.0)
    # for i in range(central_controller.num_of_quads):
    #     central_controller.quads[i].planner.set_segment_time(segment_times=[3.0, 3.0])
    #     central_controller.quads[i].solve_poly3d()
    #     central_controller.quads[i].planner.add_offset()
    #     central_controller.quads[i].load_trj_lists()

    while not rospy.is_shutdown():
        for i_controller in central_controller.quads:
            # i_controller.set_hover_des(1.5)
            # i_controller.hover_and_trj_xy(dimension='x')
            i_controller.publish_poly3d_trj()
            i_controller.publish_err()
            i_controller.update_current_state()
            i_controller.update_desired_values()
            i_controller.motorSpeedFromU()
            # i_controller.multiply_motor_speed(1.2)
            i_controller.send_motor_command()
            rate.sleep()

