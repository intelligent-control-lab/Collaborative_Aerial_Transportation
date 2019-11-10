import numpy as np
import yaml
import os
import sys
sys.path.append("/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/")
from polynomialTrjNonlinear.Optimizer_nonlinear import PolynomialOptNonlinear
from basic_functions import *


class PayloadDistributor(object):
    def __init__(self, c_init, index):
        self.index = index
        self.g = 9.81
        # for the payload
        self.mass_payload = 1.0
        self.H0_ = np.zeros((6, 6))
        self.J_payload = np.zeros((3, 3))
        # for the quad-rotors
        self.arm_length = 0.0
        self.mass_quad = 0.0
        self.num_agents = 4
        self.relative_pos = np.zeros((3, self.num_agents))
        # self.c = np.ones((self.num_agents, 1)) / self.num_agents
        self.c = c_init
        self.Sigma = np.zeros((3, 3))
        self.E_ = np.zeros((6, 6))
        self.E_inv = np.zeros((6, 6))
        self.c_E = np.zeros((6, 6))
        self.M0_ = np.zeros((6, 6))
        self.G0_ = np.zeros((6, 1))
        self.W0_ = np.zeros((6, 1))

        self.pre_processing()
        self.H0_[0:3, 0:3] = self.mass_payload * np.eye(3)
        self.H0_[3:, 3:] = self.J_payload
        self.G0_[2, 0] = self.g * self.mass_payload * 0.31
        # self.G0_ = np.multiply(self.mass_payload, self.G0_)
        self.calculateMatrixE()
        self.planner = PolynomialOptNonlinear(N=10, dimension=3)

        if self.checkValidC():
            print "Correct value on c"
        else:
            print "Wrong value on c, please retry"

    def pre_processing(self):
        filename='/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/config/parameters.yaml'
        with open(filename, 'r') as stream:
            try:
                yaml_data = yaml.safe_load(stream)

                # physical properties of the payload:
                J_base_P = box_inertia(yaml_data['payload']['base']['box']['x'],
                                            yaml_data['payload']['base']['box']['y'],
                                            yaml_data['payload']['base']['box']['z'],
                                            yaml_data['payload']['base']['mass'])
                J_holder_G = box_inertia(yaml_data['payload']['holder']['box']['x'],
                                            yaml_data['payload']['holder']['box']['y'],
                                            yaml_data['payload']['holder']['box']['z'],
                                            yaml_data['payload']['holder']['mass'])

                J_holders_O = \
                    [
                        deplacement_moment_inertia(
                            [-yaml_data['holder0']['origin']['x'],
                             -yaml_data['holder0']['origin']['y'],
                             -yaml_data['holder0']['origin']['z']], J_holder_G, yaml_data['payload']['holder']['mass']),
                        deplacement_moment_inertia(
                            [-yaml_data['holder1']['origin']['x'],
                             -yaml_data['holder1']['origin']['y'],
                             -yaml_data['holder1']['origin']['z']], J_holder_G, yaml_data['payload']['holder']['mass']),
                        deplacement_moment_inertia(
                            [-yaml_data['holder2']['origin']['x'],
                             -yaml_data['holder2']['origin']['y'],
                             -yaml_data['holder2']['origin']['z']], J_holder_G, yaml_data['payload']['holder']['mass']),
                        deplacement_moment_inertia(
                            [-yaml_data['holder3']['origin']['x'],
                             -yaml_data['holder3']['origin']['y'],
                             -yaml_data['holder3']['origin']['z']], J_holder_G, yaml_data['payload']['holder']['mass']),
                    ]
                self.mass_payload = yaml_data['payload']['base']['mass']
                mass_holder = yaml_data['payload']['holder']['mass']
                self.mass_payload = self.mass_payload + self.num_agents * mass_holder
                self.J_payload = J_base_P
                for j in range(self.num_agents):
                    self.J_payload = self.J_payload + J_holders_O[j]
                self.relative_pos = \
                    [
                        np.array([
                         yaml_data['holder0']['origin']['x'],
                         yaml_data['holder0']['origin']['y'],
                         yaml_data['quad']['base']['height']/2.0 
                         + yaml_data['payload']['base']['box']['z']/2.0 
                         + yaml_data['payload']['holder']['box']['z']
                        ]),

                        np.array([
                         yaml_data['holder1']['origin']['x'],
                         yaml_data['holder1']['origin']['y'],
                         yaml_data['quad']['base']['height']/2.0 
                         + yaml_data['payload']['base']['box']['z']/2.0 
                         + yaml_data['payload']['holder']['box']['z']
                        ]),

                        np.array([
                         yaml_data['holder2']['origin']['x'],
                         yaml_data['holder2']['origin']['y'],
                         yaml_data['quad']['base']['height']/2.0 
                         + yaml_data['payload']['base']['box']['z']/2.0 
                         + yaml_data['payload']['holder']['box']['z']
                        ]),

                        np.array([
                         yaml_data['holder3']['origin']['x'],
                         yaml_data['holder3']['origin']['y'],
                         yaml_data['quad']['base']['height']/2.0 
                         + yaml_data['payload']['base']['box']['z']/2.0 
                         + yaml_data['payload']['holder']['box']['z']
                        ])
                    ]

            except yaml.YAMLError as exc:
                print(exc)

    def update_c(self, c):
        self.c = c
        
    def checkValidC(self):
        return np.sum(self.c) == 1.0

    def updateSigma(self):
        sum = np.zeros((3, 3))
        for i in range(self.num_agents):
            S_ri = skewsymetric(self.relative_pos[i])
            sum = sum + np.multiply(self.c[i], np.dot(S_ri, S_ri.T))
        self.Sigma = np.eye(3) + sum

    def calculateMatrixE(self):
        self.E_[0:3, 0:3] = np.eye(3)
        self.E_[0:3, 3:] = np.zeros((3, 3))
        self.E_[3:, 0:3] = skewsymetric(self.relative_pos[self.index])
        self.E_[3:, 3:] = np.eye(3)

    def update_cE(self):
        self.updateSigma()
        self.E_inv[0:3, 0:3] = np.eye(3)
        self.E_inv[0:3, 3:] = -np.dot(skewsymetric(self.relative_pos[self.index]),
                                      np.linalg.inv(self.Sigma))
        self.E_inv[3:, 0:3] = np.zeros((3, 3))
        self.E_inv[3:, 3:] = np.linalg.inv(self.Sigma)
        self.c_E = np.multiply(self.c[self.index], self.E_inv)

    def updateM0W0(self):
        self.M0_[:, :] = np.dot(
            np.dot(
                self.E_inv, self.H0_
            )
            , np.linalg.inv(self.E_.T)
        )
        self.W0_ = np.dot(self.E_inv, self.G0_)

    def show_distribution(self):
        for i in range(self.num_agents):
            print "i: ", i
            print "M0: ", self.M0_
            print "W0: ", self.W0_

    def getM0(self):
        return self.M0_

    def getW0(self):
        return self.W0_

    def get_c(self, index=0):
        return self.c[index]

    def setVerticesPosVel(self, positions, velocities):
        self.planner.setVerticesPosVel(positions, velocities)

    def get_d_trajectory(self, order, sample_frequency):
        self.planner.linear_opt.get_d_trajectory(order=order, sample_frequency=sample_frequency)

    def get_acceleratons(self, frequecy):
        self.get_d_trajectory(order=2, sample_frequency=frequecy)

    def generateDeiredTrj(self, positions, velocities):
        self.setVerticesPosVel(positions, velocities)
        self.planner.optimizeTimeAndFreeConstraints()

    def plot_derivatives(self, order):
        self.planner.linear_opt.plot_derivatives(order=order, solver='nonlinear')


if __name__ == '__main__':
    distributor = PayloadDistributor()
    distributor.generateDeiredTrj(positions=[[0.0, 0.0, 0.0],
                                             [2.0, 0.0, 2.0],
                                             [4.0, 0.0, 5.0]],
                                  velocities=[[0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0]])
    distributor.get_d_trajectory(order=0, sample_frequency=50)
    distributor.get_d_trajectory(order=1, sample_frequency=50)
    distributor.get_d_trajectory(order=2, sample_frequency=50)

    distributor.update_cE()
    c_E = distributor.c_E
    distributor.updateM0W0()
    distributor.show_distribution()
    distributor.plot_derivatives(order=0)
    distributor.plot_derivatives(order=1)
    distributor.plot_derivatives(order=2)



