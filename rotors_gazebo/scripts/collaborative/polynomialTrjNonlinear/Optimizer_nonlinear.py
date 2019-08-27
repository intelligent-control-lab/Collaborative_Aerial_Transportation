import nlopt
from Optimizer_linear import PolynomialOptLinear
from optParamsNonlinear import OptNonlinearParams
from vertex import *


class PolynomialOptNonlinear(object):
    def __init__(self, N, dimension):
        self.linear_opt = PolynomialOptLinear(N, dimension=dimension)
        self.nl_opt = None
        self.inequality_constrain = []
        self.vertices = []
        self.k_time = 1.0
        self.initial_solutons = []
        self.initial_step = []
        self.order_to_optimize = 4
        self.lower_bound = []
        self.upper_bound = []
        self.lower_bound_free = []
        self.upper_bound_free = []
        self.segment_times = []
        self.lower_bound_time = 0.1

        self.nl_opt_params = OptNonlinearParams()

    def setupFromVertices(self, vertices):
        self.linear_opt.setupFromVertices(vertices=vertices)
        self.segment_times = self.linear_opt.get_segment_times()

    def addMaximumMagnitudeConstrain(self, derivative, max_value):
        constrain = ConstrainData(derivative, max_value)
        self.inequality_constrain.append(constrain)

    def solve_linear(self, opt_order):
        return self.linear_opt.solve_linear(order_to_opt=opt_order)

    def init_offset(self, offset):
        self.linear_opt.set_init_offset(offset)

    def optimizeTimeAndFreeConstraints(self):
        # solve linear optimization problem to find the initial solutions
        self.linear_opt.solve_linear(order_to_opt=self.order_to_optimize)
        self.segment_times = self.linear_opt.get_segment_times()
        free_constrains = self.linear_opt.get_free_constrains()
        n_segments = self.linear_opt.get_n_segments()
        dimension = self.linear_opt.get_dimention()
        if len(free_constrains) == 0:
            print "no free constrains, same as the time only optimization..."

        # number of variables to be optimized
        n_optimization_variables = n_segments + len(free_constrains) * dimension

        self.vertices = self.linear_opt.get_vertices()

        '''
        Update lower bound and upper bound according to
        the bounds saved in in-equal constrains
        '''
        self.lower_bound = [self.lower_bound_time for _ in range(self.linear_opt.get_n_segments())]
        self.upper_bound = [np.inf for _ in range(self.linear_opt.get_n_segments())]
        self.lower_bound_free = [-np.inf for _ in range(self.linear_opt.get_dimention() *
                                                        self.linear_opt.get_n_free_constrains())]
        self.upper_bound_free = [np.inf for _ in range(self.linear_opt.get_dimention() *
                                                        self.linear_opt.get_n_free_constrains())]

        # set free constrains and concatenate
        self.setFreeEndHardConstrains()
        self.lower_bound = self.lower_bound + self.lower_bound_free
        self.upper_bound = self.upper_bound + self.upper_bound_free

        '''
        Update the initial solutions in the following order:
        [segment_times, dps by linear_solver]
        '''
        for seg_time in self.segment_times:
            self.initial_solutons.append(seg_time)

        dp_init = self.linear_opt.get_dp()
        for dim in range(self.linear_opt.get_dimention()):
            for i_dp in dp_init[:, dim]:
                self.initial_solutons.append(i_dp)

        for i, initial_sol in enumerate(self.initial_solutons):
            self.lower_bound[i] = min(self.lower_bound[i], initial_sol)
            self.upper_bound[i] = max(self.upper_bound[i], initial_sol)

        # initial step
        for i in range(n_optimization_variables):
            self.initial_step.append(1e-13)
        '''
        Set the nonlinear solver parameters
        '''
        self.nl_opt = nlopt.opt(nlopt.LN_BOBYQA, n_optimization_variables)
        self.nl_opt.set_ftol_rel(self.nl_opt_params.f_rel)
        self.nl_opt.set_ftol_abs(self.nl_opt_params.f_abs)
        self.nl_opt.set_xtol_rel(self.nl_opt_params.x_rel)
        self.nl_opt.set_xtol_abs(self.nl_opt_params.x_abs)
        self.nl_opt.set_maxeval(self.nl_opt_params.max_interations)
        self.nl_opt.set_min_objective(self.objective_time_constrains)
        self.nl_opt.set_initial_step(self.initial_step)

        '''
        Set the lower and upper bounds
        '''
        if len(self.lower_bound) == len(self.initial_solutons):
            self.nl_opt.set_lower_bounds(self.lower_bound)
            self.nl_opt.set_upper_bounds(self.upper_bound)
        else:
            print "dismatch dimensions between bounds and variables"

        # final_cost = np.inf
        result = self.nl_opt.optimize(self.initial_solutons)

        self.linear_opt.reconstructSegmentsFromOpt()
        return result

    def setVerticesPosVel(self, positions, velocities, dim=3):
        vertices = []
        vertex_0 = Vertex(dimension=dim, index=0)
        start_position = positions[0]
        start_velocity = velocities[0]

        self.construct_constrain(vertex_0, order=0, input_list=start_position)
        self.construct_constrain(vertex_0, order=1, input_list=start_velocity)
        vertices.append(vertex_0)

        vertex_0.makeStartOrEnd(position=start_position, up_to_order=4)

        for i, position in enumerate(positions[1:-1]):
            vertex = Vertex(dimension=dim, index=1)
            velocity = velocities[i]
            self.construct_constrain(vertex, order=0, input_list=position)
            self.construct_constrain(vertex, order=1, input_list=velocity)
            vertices.append(vertex)

        vertex_2 = Vertex(dimension=dim, index=2)
        pos = positions[-1]
        vel = velocities[-1]

        self.construct_constrain(vertex_2, order=0, input_list=pos)
        self.construct_constrain(vertex_2, order=1, input_list=vel)
        vertex_2.makeStartOrEnd(position=pos, up_to_order=4)
        vertices.append(vertex_2)

        self.linear_opt.setupFromVertices(vertices)
        self.add_max_vel(2.0)
        self.add_max_acc(1.0)

    def setFreeEndHardConstrains(self):
        free_constrains = self.linear_opt.get_free_constrains()
        for i_ineq_constrain in self.inequality_constrain:
            order_ineq = i_ineq_constrain.get_order()
            value = i_ineq_constrain.get_value()
            for i, free_cons in enumerate(free_constrains):
                if free_cons.order == order_ineq:
                    self.lower_bound_free[i] = -abs(value)
                    self.upper_bound_free[i] = abs(value)

    def objective_time_constrains(self, x, cost):
        """
        Objective function to be optimized.
        :param x: the optimization variable, in the following order:
        [segment times, free constrains]
        :return:
        """
        n_segment = self.linear_opt.get_n_segments()
        n_free_constrain = self.linear_opt.get_n_free_constrains()
        dim = self.linear_opt.get_dimention()
        self.segment_times = self.linear_opt.get_segment_times()

        if not x.size == n_segment + dim * n_free_constrain:
            print "x has the wrong size in the objective function.Stop."
            return

        '''
        Upgrade the variables from input x
        '''
        free_constrains = np.zeros((n_free_constrain, dim))
        segment_times = []

        for i in range(n_segment):
            segment_times.append(x[i])

        for i_dim in range(dim):
            for i in range(n_free_constrain):
                free_constrains[i, i_dim] = x[i + self.linear_opt.get_n_segments() + i_dim*n_free_constrain]

        self.linear_opt.set_dp(free_constrains)
        self.linear_opt.set_seg_times(segment_times)

        '''
        Compute trajectory costs
        '''
        cost_trajectory = self.linear_opt.get_cost_total(self.order_to_optimize)

        '''
        Compute segment time cost
        '''
        total_time = 0.0
        for i, seg_time in enumerate(self.linear_opt.get_segment_times()):
            total_time = total_time + seg_time

        cost_time = total_time * total_time * self.k_time
        cost = cost_trajectory + cost_time

        # print "cost: ", cost
        return cost

    def add_max_vel(self, max_v):
        self.addMaximumMagnitudeConstrain(derivative=1, max_value=max_v)

    def add_max_acc(self, max_a):
        self.addMaximumMagnitudeConstrain(derivative=2, max_value=max_a)

    def construct_constrain(self, vertex, order, input_list):
        self.linear_opt.construct_constrain(vertex=vertex, order=order, input_list=input_list)

    def constrain_from_arr(self, vertex, order, arr):
        if max(arr.shape[0], arr.shape[1]) == self.linear_opt.D_:
            vertex.add_constrain(derivative_order=order, value=arr.reshape((self.linear_opt.D_, 1)))

    def estimate_segment_time(self):
        return self.linear_opt.estimate_segment_time()


if __name__ == '__main__':
    dimension = 3
    opt_nl = PolynomialOptNonlinear(N=10, dimension=dimension)
    opt_nl.setVerticesPosVel(positions=[[0.0, 0.0, 0.0],
                                        [2.0, 0.0, 2.0],
                                        [4.0, 0.0, 5.0]],
                             velocities=[[0.0, 0.0, 0.0],
                                         [0.0, 0.0, 0.0],
                                         [0.0, 0.0, 0.0]])
    result = opt_nl.optimizeTimeAndFreeConstraints()

    print "result: ", result
    opt_nl.linear_opt.get_d_trajectory(order=0, sample_frequency=50)
    opt_nl.linear_opt.get_d_trajectory(order=1, sample_frequency=50)
    opt_nl.linear_opt.get_d_trajectory(order=2, sample_frequency=50)
    opt_nl.linear_opt.get_d_trajectory(order=3, sample_frequency=50)
    opt_nl.linear_opt.get_d_trajectory(order=4, sample_frequency=50)
    opt_nl.linear_opt.plot_derivatives(order=0, solver='nonlinear')
    opt_nl.linear_opt.plot_derivatives(order=1, solver='nonlinear')
    opt_nl.linear_opt.plot_derivatives(order=2, solver='nonlinear')
    opt_nl.linear_opt.plot_derivatives(order=3, solver='nonlinear')
    opt_nl.linear_opt.plot_derivatives(order=4, solver='nonlinear')


