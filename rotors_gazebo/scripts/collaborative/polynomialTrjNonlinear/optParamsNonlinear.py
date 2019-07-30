import numpy as np
import nlopt
from enum import Enum


class OptNonlinearParams(object):
    def __init__(self):
        self.f_abs = -1
        self.f_rel = 0.05
        self.x_abs = -1
        self.x_rel = -1
        self.initial_stepsize_rel = 0.1
        self.equality_constrain_tolerence = 10e-3
        self.inequality_constrain_tolerence = 0.001
        self.max_interations = 3000
        self.time_penalty = 500.0

        self.algorithm = nlopt.LN_BOBYQA

        self.random_seed = 0

        self.use_soft_constrains = True
        self.soft_constrain_weight = 100.0

        self.TimeAllocMethod = TimeAllocMethod

    def get_time_alloc_method(self):
        return self.TimeAllocMethod


class TimeAllocMethod(Enum):
    KSquaredTime = 1
    kRichterTime = 2
    kMellingerOuterLoop = 3
    kSquaredTimeAndConstraints = 4
    kRichterTimeAndConstraints = 5
    kUnknown = 6


class OptimizationNonlinearInfo(object):
    def __init__(self, Nlopt):
        self.opt_nl = Nlopt
        self.n_interations = 0
        self.stopping_reason = self.opt_nl.FAILURE


if __name__ == '__main__':
    opt_nonlinear_params = OptNonlinearParams()
    t_alloc = opt_nonlinear_params.get_time_alloc_method().KSquaredTime

