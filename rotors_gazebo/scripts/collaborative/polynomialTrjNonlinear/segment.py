import numpy as np
from polynomial import Polynomial
from matplotlib import pyplot as plt


class Segment(object):
    def __init__(self, seg_time, dimension, N):
        self.Time = seg_time
        self.N_ = N
        self.D_ = dimension
        self.polynomials = [Polynomial(self.N_) for _ in range(self.D_)]
        self.Q = np.zeros((self.N_, self.N_))
        self.Qs = [np.zeros((self.N_, self.N_)) for _ in range(self.N_)]
        self.coeffs = np.zeros((self.N_, self.D_))

        # samples
        self.t = []
        self.samples_derivatives = []
        self.offset = []

    def set_t_array(self, num_intevals):
        t = np.linspace(start=0.0, stop=self.Time, num=num_intevals)
        self.t = t

    def set_offset(self, offset):
        self.offset = offset

    def get_offset(self):
        return self.offset

    def sample_derivative_value(self, order, num_intevals):
        self.set_t_array(num_intevals)
        self.samples_derivatives = np.zeros((self.t.shape[0], self.D_))
        if order > self.N_ - 1:
            print "Derivative order larger than the maximum order of the polynomial, zeros"
            self.samples_derivatives = np.zeros((self.t.shape[0], self.D_))
        else:
            ts = np.array([pow(self.t, i) for i in range(self.N_ - order)]).transpose()
            for i_dim in range(self.D_):
                coef_derivatives = list(self.coeffs[order:self.N_, i_dim])
                for j in range(self.N_ - order):
                    coef_derivatives[j] = coef_derivatives[j] * np.math.factorial(j+order) / np.math.factorial(j)

                self.samples_derivatives[:, i_dim] = np.dot(ts, coef_derivatives)

    def get_polynomials(self):
        return self.polynomials

    def set_coefficients(self, coeffs):
        self.coeffs = coeffs

    def get_coefficients(self):
        return self.coeffs

    def computeCostMatrix(self, derivative):
        for i in range(derivative, self.N_):
            for l in range(derivative, self.N_):
                self.Q[i, l] = 2 * np.math.factorial(i) / np.math.factorial(i-derivative) * \
                               np.math.factorial(l) / np.math.factorial(l-derivative) * \
                               pow(self.Time, i + l - 2*derivative + 1) / (i + l - 2*derivative + 1)

    def computeCostForAllDerivatives(self):
        for r in range(self.N_):
            self.computeCostMatrix(r)

    def chooseQ(self, derivativeToOpt):
        zeros = np.zeros((1, self.N_))
        zeros[0, derivativeToOpt] = 1.0
        self.Q = np.multiply(self.Qs[0], zeros[0])
        for i in range(1,self.N_):
            self.Q = self.Q + np.multiply(self.Qs[i], zeros[i])

    def get_endTime(self):
        return self.Time

    def set_segment_time(self, time):
        self.Time = time

    def get_Q(self):
        return self.Q

    def plot_derivatives(self, order, num_intervals):
        self.sample_derivative_value(order=order, num_intevals=num_intervals)
        for i_dim in range(self.D_):
            if order == 1:
                plt.title('dim%d: VELOCITY' %i_dim)
            elif order == 2:
                plt.title('dim%d: ACCELERATION' %i_dim)
            elif order == 3:
                plt.title('dim%d: JERK' %i_dim)
            elif order == 4:
                plt.title('dim%d: SNAP' %i_dim)

            plt.plot(self.t, self.samples_derivatives[:, i_dim])
            plt.grid(True)
            plt.show()


if __name__ == '__main__':
    seg = Segment(seg_time=1.0, dimension=1, N=4)
    seg.computeCostForAllDerivatives()
