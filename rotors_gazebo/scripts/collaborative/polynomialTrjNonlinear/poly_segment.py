import numpy as np
from cvxopt import matrix
from cvxopt import solvers
from matplotlib import pyplot as plt
import nlopt


class PolySegment(object):
    def __init__(self, max_order, constrain_orders=[0, 1]):
        # maximum order of the polynomial
        self.max_order = max_order
        self.variable_length = max_order + 1

        # variables to select: the coefficients of the polynomial
        self.poly_coefs = np.ones((self.variable_length, 1))  # [p0, p1, p2, p3, p4, p5]

        # final cost function
        self.J = 0.0

        # Hessian Matrices for all the derivatives
        self.Qs = [np.zeros((self.variable_length, self.variable_length)) for _ in range(self.variable_length)]
        self.Penalty_c = np.zeros((self.variable_length, 1))

        # Optimization matrices
        self.Q = np.zeros((self.variable_length, self.variable_length))
        self.Q_sym = np.zeros((self.variable_length, self.variable_length))
        self.q = np.zeros((self.variable_length, 1))

        # bond constrains: on two parts, the initial constrain and the end-point constrain
        self.A = []
        self.b = []
        self.G = np.zeros((self.variable_length, self.variable_length))
        self.h = 0.0

        self.endTime = 0.0

        # constrains, default constrain on position and velocity
        self.constrains = DynamicsConstrain(constrain_orders=constrain_orders)

        # sample points
        self.samples = []
        self.samples_derivatives = []
        self.t = np.linspace(start=0.0, stop=self.endTime, num=2)

    def set_constrains(self, orders, zero_values, endtime_values):
        self.constrains.orders = orders
        self.constrains.constrain_zeroValues = zero_values
        self.constrains.constrain_endValues = endtime_values

    def set_endTime(self, endtime):
        self.endTime = endtime

    def set_A_b(self):
        for i, order_r in enumerate(self.constrains.orders):
            # for the r_th order that is going to be taken a derivative:
            A_tmp = np.zeros((2, self.variable_length))
            A_tmp[0, order_r] = np.math.factorial(order_r)

            for n in range(order_r, self.variable_length):
                A_tmp[1, n] = np.math.factorial(n) / np.math.factorial(n - order_r) * \
                              pow(self.endTime, n-order_r)
            if len(self.A) == 0:
                self.A = A_tmp
            else:
                self.A = np.concatenate((self.A, A_tmp), axis=0)

            b_tmp = np.zeros((2, 1))
            b_tmp[0, 0] = self.constrains.constrain_zeroValues[i]
            b_tmp[1, 0] = self.constrains.constrain_endValues[i]

            if len(self.b) == 0:
                self.b = b_tmp
            else:
                self.b = np.concatenate((self.b, b_tmp), axis=0)

    def set_Q_r(self, r):
        # tmp_Qr = np.zeros((self.variable_length, self.variable_length))
        for i in range(r, self.variable_length):
            for l in range(r, self.variable_length):
                self.Qs[r][i, l] = 2 * np.math.factorial(i) / np.math.factorial(i-r) * \
                               np.math.factorial(l) / np.math.factorial(l-r) * \
                               pow(self.endTime, i + l - 2*r + 1) / (i + l - 2*r + 1)

    def set_initial_values(self, init_values=None):
        if init_values is None:
            self.poly_coefs = np.random.randn(self.variable_length, 1)
        else:
            self.poly_coefs = init_values

    def set_penalties(self, penalty_list):
        for i, penalty in enumerate(penalty_list):
            self.Penalty_c[i, 0] = penalty

    def update_Q(self):
        for r in range(self.variable_length):
            self.set_Q_r(r)

        # clear Q
        self.Q = np.zeros((self.variable_length, self.variable_length))

        # load Q
        for i in range(self.variable_length):
            self.Q = self.Q + np.multiply(self.Penalty_c[i, 0], self.Qs[i])
        self.Q_sym = (self.Q.T + self.Q) / 2

    def solve(self):
        self.set_A_b()
        self.update_Q()
        self.poly_coefs = self.cvxopt_solve_qp(q=self.q, h=None, G=None)
        print "value at T: ", np.dot(self.A[1, :], self.poly_coefs)
        print "optimal poly coeffs: ", self.poly_coefs

    def plot_trj_sample(self):
        self.sample(num_intevals=100)
        plt.plot(self.t, self.samples)
        plt.grid(True)
        plt.show()

    def get_endTime(self):
        return self.endTime

    def get_poly_coeffs(self):
        return self.poly_coefs

    def cvxopt_solve_qp(self, q=None, G=None, h=None):
        P = matrix(self.Q_sym)
        q = matrix(q)
        if G is not None:
            G = matrix(G)
        if self.A is not None:
            A = matrix(self.A)
            b = matrix(self.b)
        sol = solvers.qp(P=P, q=q, G=G, h=h, A=A, b=b)
        if 'optimal' not in sol['status']:
            return None
        return np.array(sol['x']).reshape((self.Q.shape[1],))

    def set_t_array(self, num_intevals):
        t = np.linspace(start=0.0, stop=self.endTime, num=num_intevals)
        self.t = t

    def sample(self, num_intevals):
        self.set_t_array(num_intevals=num_intevals)
        ts = np.array([pow(self.t, i) for i in range(self.variable_length)]).transpose()
        self.samples = np.dot(ts, self.poly_coefs)

    def sample_derivative_value(self, order, num_intevals):
        self.set_t_array(num_intevals)
        if order > self.max_order:
            print "Derivative order larger than the maximum order of the polynomial, zeros"
            self.samples_derivatives = np.zeros(self.t.shape)
        else:
            ts = np.array([pow(self.t, i) for i in range(self.variable_length - order)]).transpose()
            coef_derivatives = self.poly_coefs[order:self.variable_length]
            for j in range(self.variable_length - order):
                coef_derivatives[j] = coef_derivatives[j] * np.math.factorial(j+order) / np.math.factorial(j)

            self.samples_derivatives = np.dot(ts, coef_derivatives)

    def plot_derivatives(self, order, num_intervals):
        self.sample_derivative_value(order=order, num_intevals=num_intervals)
        plt.plot(self.t, self.samples_derivatives)
        plt.grid(True)
        plt.show()

    def run(self):
        self.set_initial_values()
        self.solve()

    def set_objectives_penalty(self, mode="snap"):
        if mode == "snap":
            l_penalty = np.zeros((1, self.variable_length))[0, :].tolist()
            l_penalty[4] = 1.0
            self.set_penalties(l_penalty)
        if mode == "jerk":
            l_penalty = np.zeros((1, self.variable_length))[0, :].tolist()
            l_penalty[3] = 1.0
            self.set_penalties(l_penalty)
        if mode == "acc":
            l_penalty = np.zeros((1, self.variable_length))[0, :].tolist()
            l_penalty[2] = 1.0
            self.set_penalties(l_penalty)
        if mode == "vel":
            l_penalty = np.zeros((1, self.variable_length))[0, :].tolist()
            l_penalty[1] = 1.0
            self.set_penalties(l_penalty)
        if mode == "avg":
            l_penalty = np.ones((1, self.variable_length))
            l_penalty = l_penalty[0, :] / sum(l_penalty)
            self.set_penalties(l_penalty)


class DynamicsConstrain(object):
    def __init__(self, constrain_orders):
        self.orders = constrain_orders
        self.constrain_zeroValues = [0.0 for _ in range(len(constrain_orders))]
        self.constrain_endValues = [0.0 for _ in range(len(constrain_orders))]

    def set_zeroValues(self, zero_values):
        self.constrain_zeroValues = zero_values

    def set_entValues(self, endValues):
        self.constrain_endValues = endValues


if __name__ == "__main__":
    segment = PolySegment(max_order=7, constrain_orders=[0, 1])
    segment.set_constrains(
        orders=[0, 1, 2, 3],
        zero_values=[3.0, 0.0, 0.0, 0.0],
        endtime_values=[6.0, 0.0, 0.0, 0.0]
    )
    # segment.set_penalties(penalty_list=[1.0/8, 1.0/8, 1.0/8, 1.0/8, 1.0/8, 1.0/8, 1.0/8, 1.0/8])
    segment.set_objectives_penalty(mode="vel")
    segment.set_endTime(endtime=2.0)
    segment.run()
    segment.plot_trj_sample()
    segment.sample_derivative_value(order=1, num_intevals=100)
    segment.plot_derivatives(order=1, num_intervals=100)

    # [  3.00000000e+00   9.09938791e-13  -1.42108547e-13   6.31973910e-15
    # 6.56250000e+00  -7.87500000e+00   3.28125000e+00  -4.68750000e-01]