import numpy as np


class Polynomial(object):
    def __init__(self, N):
        '''
        :param N: N coefficients, order N-1
        :param coefficients: value of the coefficients
        '''

        self.N = N
        self.coefficients = np.array([[1] for i in range(self.N)])
        self.base_coefficients_ = np.zeros((N, N))
        self.calculateBaseCoefficients()

    def valueWithTime(self, t):
        ts = np.array([pow(t, i) for i in range(self.N)])
        return np.dot(ts, self.coefficients)[0]

    def calculateBaseCoefficients(self):
        self.base_coefficients_[0, :] = 1.0
        for i in range(1, self.N):
            for j in range(i, self.N):
                self.base_coefficients_[i, j] = self.base_coefficients_[i-1, j] * (j - i + 1)

    def set_coefficients(self, coefficients):
        self.coefficients = coefficients


if __name__ == '__main__':
    poly = Polynomial(4)
    value = poly.valueWithTime(2)
    poly.calculateBaseCoefficients()
