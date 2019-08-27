import numpy as np
from vertex import Vertex, Constrain
from scipy.sparse import coo_matrix
from segment import Segment
from polynomial import Polynomial
import scipy.linalg.decomp_qr as QR
from matplotlib import pyplot as plt


class PolynomialOptLinear(object):
    def __init__(self, N, dimension):
        self.n_vertices = 0
        self.n_segments = 0
        self.N_ = N
        self.maxOrderToBeConstrained = N / 2 - 1
        self.D_ = dimension
        self.cost_Matrices = []
        self.MappingMatrices = []
        self.InverseMappingMatrices = []
        self.R = []
        self.Q = []
        self.polynomial = Polynomial(N)
        self.reordering_matrix = []
        self.n_fixed_constrains = 0
        self.n_free_constrains = 0
        self.n_all_constrains = 0
        self.vertices = []
        self.segments = []
        self.df_compact = np.zeros((1, dimension))
        self.dp_compact = np.zeros((1, dimension))
        self.free_constrains = []
        self.fixed_constrains = []
        self.d_compact = []
        self.new_d_in_order = []
        self.seg_times = []

        # for plot curves:
        self.poly_pos = []
        self.poly_velocity = []
        self.poly_acc = []
        self.poly_jerk = []
        self.poly_snap = []
        self.offset_segs = []
        self.offset_all = []

        # limit on dynamics
        self.max_v = 2.0
        self.max_a = 2.0

    def get_vertices(self):
        return self.vertices

    def setupMappingMatrices(self):
        self.MappingMatrices = [np.zeros((self.N_, self.N_)) for _ in range(self.n_segments)]
        for i, seg in enumerate(self.segments):
            A = np.zeros((self.N_, self.N_))
            for j in range(self.N_/2):
                A[j, j] = self.polynomial.base_coefficients_[j, j]
            for r in range(self.N_/2):
                j = r + self.N_ / 2
                t_pows = np.array(
                    [pow(seg.Time, n) for n in range(self.N_ - r)]
                ).reshape((1, self.N_-r))
                if r > 0:
                    t_pows = np.hstack((np.zeros((1, r)), t_pows))
                A[j, :] = np.multiply(self.polynomial.base_coefficients_[r, :], t_pows)
            self.MappingMatrices[i] = A
            self.InverseMappingMatrices.append(np.zeros((self.N_, self.N_)))

    def set_offset(self):
        for i, vertex in enumerate(self.vertices[:self.n_vertices-1]):
            position_constrain = vertex.get_constrain(derivative_order=0)
            self.segments[i].set_offset(position_constrain)

    def set_init_offset(self, offset):
        self.offset_all = offset
        self.poly_pos += offset

    def setupFromVertices(self, vertices):
        self.checkValidVertices(vertices)
        self.vertices = vertices
        self.n_vertices = len(vertices)
        self.n_segments = self.n_vertices - 1
        self.seg_times = [0.0 for _ in range(self.n_segments)]
        self.segments = [Segment(1.0, self.D_, self.N_) for _ in range(self.n_segments)]
        self.seg_times = self.estimate_segment_time()
        self.set_offset()
        self.setupMappingMatrices()
        self.computeInverseA()

    def get_n_segments(self):
        return self.n_segments

    def get_dimention(self):
        return self.D_

    def checkValidVertices(self, vertices):
        """
        The constrained order cannot be larger than N/2 - 1
        :param vertices:
        :return: None
        """
        for i, vertex in enumerate(vertices):
            for order in vertex.get_constrain_orders():
                if order > self.maxOrderToBeConstrained:
                    print "invalid constrain in vertex ", vertex.get_index(), "on order: ", order
                    vertex.delete_constrain(order)

    def computeInverseA(self):
        '''
        A has the form:
        x 0 0 0 0 0
        0 x 0 0 0 0
        0 0 x 0 0 0
        0 x x x x x
        0 0 x x x x
        0 0 0 x x x
        [ A_diag B=0 ]
        [ C      D   ]
        Use the Schur-complement, the inverse is:
        [ inv(A_diag)               0      ]
        [ -inv(D) * C * inv(A_diag) inv(D) ]
        '''
        for i_A in range(self.n_segments):
            half_n = self.N_/2
            inv_res = np.zeros((self.N_, self.N_))
            A = self.MappingMatrices[i_A]
            A_diag = A[0:half_n, 0:half_n]
            C = A[half_n:self.N_, 0:half_n]
            D = A[half_n:self.N_, half_n:self.N_]
            D_inv = np.linalg.inv(D)
            inv_res[0:half_n, :] = \
                np.hstack(
                    (np.linalg.inv(A_diag), np.zeros((half_n, half_n)))
                )
            inv_res[half_n:self.N_, :] = \
                np.hstack(
                    (-np.dot(np.dot(D_inv, C), np.linalg.inv(A_diag)), D_inv)
                )
            self.InverseMappingMatrices[i_A] = inv_res

    def computeCost(self, derivativeToOpt):
        for i, seg in enumerate(self.segments):
            seg.computeCostMatrix(derivativeToOpt)
            self.Q.append(seg.Q)

    def computeR(self):
        rows = []
        cols = []
        data = []
        for i_seg in range(self.n_segments):
            A_inv = self.InverseMappingMatrices[i_seg]
            Q = self.Q[i_seg]
            H = np.dot(np.dot(A_inv.T, Q), A_inv)
            for i_row in range(self.N_):
                for i_col in range(self.N_):
                    rows.append(i_seg*self.N_ + i_row)
                    cols.append(i_seg*self.N_ + i_col)
                    data.append(H[i_row, i_col])

        self.R = coo_matrix((data, (rows, cols)), dtype=np.float64).toarray()
        self.R = np.dot(np.dot(self.reordering_matrix.T, self.R), self.reordering_matrix)

    def unique_constrain(self, constrain, constrain_list):
        for i_constrain in constrain_list:
            if constrain.equal(i_constrain):
                return False
        return True

    def computeReorderingMatrix(self):
        '''
        Reordering matrix is a permute matrix which
        maps the [df, dp] back to their initial order
        '''
        all_constrains = []

        for i, vertex in enumerate(self.vertices):
            occurance = 2
            if i == 0 or i == self.n_vertices - 1:
                occurance = 1
            for o in range(occurance):
                for r in range(self.N_ / 2):
                    constrain_r = Constrain(order=r, vertex_indx=i, dimension=self.D_)
                    constrain_r_value = vertex.get_constrain(r)
                    if constrain_r_value is None:
                        constrain_r.set_constrain(r, np.array([None for _ in range(self.D_)]))
                        if len(self.free_constrains) == 0:
                            self.free_constrains.append(constrain_r)
                        elif self.unique_constrain(constrain_r, self.free_constrains):
                            self.free_constrains.append(constrain_r)
                        all_constrains.append(constrain_r)
                    else:
                        constrain_r.set_constrain(r, constrain_r_value)
                        if len(self.fixed_constrains) == 0:
                            self.fixed_constrains.append(constrain_r)
                        elif self.unique_constrain(constrain_r, self.fixed_constrains):
                            self.fixed_constrains.append(constrain_r)
                        all_constrains.append(constrain_r)

        self.n_fixed_constrains = len(self.fixed_constrains)
        self.n_free_constrains = len(self.free_constrains)
        self.n_all_constrains = len(all_constrains)
        self.dp_compact = np.zeros((self.n_free_constrains, self.D_))

        print "n_fixed_constrains: ", self.n_fixed_constrains
        print "n_free_constrains: ", self.n_free_constrains
        print "n_all_constrains: ", self.n_all_constrains

        if self.n_fixed_constrains + self.n_free_constrains != self.n_vertices * self.N_ /2:
            print "Wrong number of fixed or free constrains! "
            return False

        # compact all the fixed constrain of all the vertex in one array,
        # for the conveniently solving the equation
        self.df_compact = np.zeros((self.n_fixed_constrains, self.D_))
        rows = []
        cols = []
        data = []

        isFixedConstrain = False
        for i, constrain in enumerate(all_constrains):
            # Here the order of M has changed to first compact fixed constrain and then free constrain order.
            for j_fix, fix_cons in enumerate(self.fixed_constrains):
                if constrain.equal(fix_cons):
                    self.df_compact[j_fix] = fix_cons.get_constrain_value()
                    rows.append(i)
                    cols.append(j_fix)
                    data.append(1.0)
                    isFixedConstrain = True

            if isFixedConstrain:
                isFixedConstrain = False
                continue
            else:
                for j_free, free_cons in enumerate(self.free_constrains):
                    if constrain.equal(free_cons):
                        rows.append(i)
                        cols.append(self.n_fixed_constrains + j_free)
                        data.append(1.0)

        self.reordering_matrix = \
            coo_matrix((data, (rows, cols)),
                shape=(
                    (self.n_all_constrains, self.n_fixed_constrains + self.n_free_constrains)
                )
            ).toarray()

        self.d_compact = np.concatenate((self.df_compact, self.dp_compact), axis=0)

    def solve_linear(self, order_to_opt):
        self.computeCost(order_to_opt)
        self.computeReorderingMatrix()
        self.computeR()
        for i in range(self.D_):
            R_pf = self.R[self.n_fixed_constrains:, 0:self.n_fixed_constrains]
            R_pp = self.R[self.n_fixed_constrains:, self.n_fixed_constrains:]
            df = self.df_compact[:, i]
            R_pp_q, R_pp_r = QR.qr(R_pp)

            RHS = np.dot(R_pf, df)

            df = np.dot(np.dot(np.linalg.inv(R_pp_r), R_pp_q.T), RHS)

        return True

    def get_segment_times(self):
        return [seg.Time for seg in self.segments]

    def get_n_free_constrains(self):
        return self.n_free_constrains

    def get_free_constrains(self):
        return self.free_constrains

    def get_fixed_constrains(self):
        return self.fixed_constrains

    def set_free_constrains(self, free_constrains):
        self.free_constrains = free_constrains

    def get_dp(self):
        return self.dp_compact

    def set_dp(self, dp):
        self.dp_compact = dp

    def get_n_fixed_constrains(self):
        return self.n_fixed_constrains

    def reconstructSegmentsFromOpt(self):
        self.d_compact = np.concatenate((self.df_compact, self.dp_compact), axis=0)
        self.new_d_in_order = np.dot(self.reordering_matrix, self.d_compact)
        for i, seg in enumerate(self.segments):
            coeffs = np.dot(self.InverseMappingMatrices[i],
                            self.new_d_in_order[i*self.N_: (i+1)*self.N_, :])
            seg.set_coefficients(coeffs)
            seg.Time = self.seg_times[i]

    def get_cost_total(self, order_to_optimize):
        self.d_compact = np.concatenate((self.df_compact, self.dp_compact), axis=0)
        self.computeCost(derivativeToOpt=order_to_optimize)
        self.computeR()
        cost = 0.0
        for i_dim in range(self.D_):
            d_compact_one_dim = self.d_compact[:, i_dim]
            cost += np.dot(np.dot(d_compact_one_dim.T, self.R), d_compact_one_dim)
        return cost

    def get_segments(self):
        return self.segments

    def set_seg_times(self, segtimes):
        self.seg_times = segtimes

    def get_d_trajectory(self, order, sample_frequency):
        total_samples = 0
        for i, seg in enumerate(self.segments):
            num_intervals = int(sample_frequency * seg.get_endTime())
            seg.sample_derivative_value(order=order, num_intevals=num_intervals)
            self.segments[i].sample_derivative_value(order=order, num_intevals=num_intervals)
            if i == 0:
                trj_value = self.segments[i].samples_derivatives
            else:
                trj_value = np.concatenate(
                    (trj_value, self.segments[i].samples_derivatives)
                )
            total_samples += num_intervals
        trj_value = trj_value.reshape((total_samples, self.D_))
        if order == 0:
            self.poly_pos = trj_value
        if order == 1:
            self.poly_velocity = trj_value
        elif order == 2:
            self.poly_acc = trj_value
        elif order == 3:
            self.poly_jerk = trj_value
        elif order == 4:
            self.poly_snap = trj_value
        return trj_value

    def plot_derivatives(self, order, solver='linear'):
        if order == 0:
            plt.plot(self.poly_pos)
            plt.title('POSITION_%s_solver' %solver)
            plt.legend('xyz')
        if order == 1:
            plt.plot(self.poly_velocity)
            plt.title('VELOCITY_%s_solver' %solver)
            plt.legend('xyz')
        elif order == 2:
            plt.plot(self.poly_acc)
            plt.title('ACCELERATION_%s_solver' %solver)
            plt.legend('xyz')
        elif order == 3:
            plt.plot(self.poly_jerk)
            plt.title('JERK_%s_solver' %solver)
            plt.legend('xyz')
        elif order == 4:
            plt.plot(self.poly_snap)
            plt.title('SNAP_%s_solver' %solver)
            plt.legend('xyz')
        plt.grid(True)
        plt.show()

    def construct_constrain(self, vertex, order, input_list):
        if order <= self.maxOrderToBeConstrained and len(input_list) == self.D_:
            value_array = np.array([input_list[i] for i in range(self.D_)])
            vertex.add_constrain(derivative_order=order, value=value_array)

    def estimate_segment_time(self, constant=6.5):
        '''
        refer: eth_mav_trajectory_generation project
        '''
        if len(self.vertices) != 0:
            seg_times = []
            vertex_0 = list(self.vertices)[0]
            for i, vertex in enumerate(self.vertices[1:]):
                pos_front = vertex_0.get_constrain(derivative_order=0)
                pos_next = vertex.get_constrain(derivative_order=0)
                pos_diff = (pos_next - pos_front)
                vertex_0 = vertex
                dist = np.sqrt(np.dot(pos_diff, pos_diff))
                t = dist / self.max_v * 2 * (1.0 + constant * self.max_v / self.max_a * np.exp(-dist / self.max_v * 2))
                seg_times.append(t)
                self.segments[i].set_segment_time(t)
            return seg_times
        else:
            print "No vertices yet, setup vertices first!"
            return None


if __name__ == '__main__':
    dimension = 3
    opt = PolynomialOptLinear(N=10, dimension=dimension)
    vertices = []
    vertex0 = Vertex(dimension=dimension, index=0)
    start_position = [0.0, 0.0, 0.0]
    start_velocity = [0.0, 0.0, 0.0]
    opt.construct_constrain(vertex0, order=0, input_list=start_position)
    opt.construct_constrain(vertex0, order=1, input_list=start_velocity)

    vertex0.makeStartOrEnd(position=start_position, up_to_order=4)

    vertex1 = Vertex(dimension=dimension, index=1)

    position = [2.0, 0.0, 2.0]
    velocity = [0.0, 0.0, 0.0]
    opt.construct_constrain(vertex1, order=0, input_list=position)
    opt.construct_constrain(vertex1, order=1, input_list=velocity)

    vertex2 = Vertex(dimension=dimension, index=2)
    position = [4.0, 0.0, 4.0]
    velocity = [0.0, 0.0, 0.0]
    opt.construct_constrain(vertex2, order=0, input_list=position)
    opt.construct_constrain(vertex2, order=1, input_list=velocity)

    vertex2.makeStartOrEnd(position=position, up_to_order=4)

    vertices.append(vertex0)
    vertices.append(vertex1)
    vertices.append(vertex2)

    opt.setupFromVertices(vertices)

    opt.solve_linear(order_to_opt=4)
    aa = opt.dp_compact
    opt.reconstructSegmentsFromOpt()
    segs = opt.get_segments()
    # opt.plotFromSegments()
    opt.get_d_trajectory(order=0, sample_frequency=50)
    opt.get_d_trajectory(order=1, sample_frequency=50)
    opt.get_d_trajectory(order=2, sample_frequency=50)
    opt.get_d_trajectory(order=3, sample_frequency=50)
    opt.get_d_trajectory(order=4, sample_frequency=50)
    opt.plot_derivatives(order=0)
    opt.plot_derivatives(order=1)
    opt.plot_derivatives(order=2)
    opt.plot_derivatives(order=3)
    opt.plot_derivatives(order=4)
    dd = 0





