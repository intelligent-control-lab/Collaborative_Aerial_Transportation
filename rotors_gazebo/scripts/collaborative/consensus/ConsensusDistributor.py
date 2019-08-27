import numpy as np
from matplotlib import pyplot as plt


class Edge(object):
    def __init__(self, start, end, weight):
        self.start = start
        self.end = end
        self.weight = weight

    def set_start(self, start_index):
        self.start = start_index

    def set_end(self, end_index):
        self.end = end_index

    def set_weight(self, weight):
        self.weight = weight

    def get_start(self):
        return self.start

    def get_end(self):
        return self.end

    def get_weight(self):
        return self.weight


class ConsensusAgent(object):
    def __init__(self, index, initial_guess):
        self.num_agents = 4
        self.c = initial_guess
        self.index = index
        self.neighbours = [False for _ in range(self.num_agents)]
        self.value = 0.0


class ConsensusNet(object):
    def __init__(self, num_agents, num_edges):
        self.num_agents = num_agents
        self.num_edges = num_edges
        self.W = np.zeros((num_agents, num_agents))
        self.incidence_matrix = np.zeros((num_agents, num_edges))
        self.agents = [ConsensusAgent(i, 0.0) for i in range(self.num_agents)]
        self.values = np.array([agent.value for agent in self.agents]).reshape((self.num_agents, 1))
        self.set_init_values([0.05, 0.45, 0.13, 0.37])
        self.values_history = self.values.copy()
        self.avg_weight = 0.25
        self.edges = [Edge(i, i+1, self.avg_weight) for i in range(self.num_agents-1)]
        self.edges.append(Edge(self.num_agents-1, 0, self.avg_weight))
        self.weights = np.zeros((self.num_edges, 1))
        self.diag_weight = np.zeros((self.num_edges, self.num_edges))
        self.set_incidence_matrix()
        self.init_edge_weights()
        self.init_W()

    def set_incidence_matrix(self):
        for i_edge, edge in enumerate(self.edges):
            i_start = edge.get_start()
            i_end = edge.get_end()
            self.incidence_matrix[i_start, i_edge] = 1
            self.incidence_matrix[i_end, i_edge] = -1

    def init_edge_weights(self):
        for i, edge in enumerate(self.edges):
            self.weights[i] = edge.get_weight()
            self.diag_weight[i, i] = edge.get_weight()

    def init_W(self):
        self.W = np.eye(self.num_agents) - np.dot(np.dot(self.incidence_matrix, self.diag_weight),
                                                  self.incidence_matrix.T)

    def set_init_values(self, init_values):
        self.values = np.array(init_values).reshape((self.num_agents, 1))

    def update_values(self):
        new_value = np.dot(self.W, self.values).copy()
        self.values = new_value
        self.values_history = np.concatenate((self.values_history, new_value), axis=1)
        return self.values

    def get_value(self, index):
        return self.values[index, 0]

    def plot_consensus_history(self):
        plt.plot(self.values_history.T)
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    net = ConsensusNet(4, 4)
    net.set_incidence_matrix()

    print "incidence_matrix: ", net.incidence_matrix
    print "W: ", net.W
    for i in range(100):
        net.update_values()

    aa = 0
    plt.plot(net.values_history.T)
    plt.grid(True)
    plt.show()



