from mellinger_trj_nlopt import *
from payload_distributor import *


class MellingerWithDistributor(Mellinger):
    def __init__(self, mav_name, x, y, z, dim=3):
        Mellinger.__init__(mav_name, x, y, z, dimension=dim)
        self.M = np.zeros()

    def update_distributions(self, M_0, W_0):
        self.M =

