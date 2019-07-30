import numpy as np


class Vertex(object):
    def __init__(self, dimension, index):
        self.dimension = dimension
        self.constrain_value = np.zeros((1, self.dimension))
        self.Constraints = dict()
        self.index = index

    def set_index(self, indx):
        self.index = indx

    def get_index(self):
        return self.index

    def add_constrain(self, derivative_order, value):
        if value.shape[0] != self.dimension:
            print "Wrong dimension of the constrain! "
            return False
        self.Constraints[derivative_order] = value

    def has_constrain(self, derivative_order):
        return derivative_order in self.Constraints.keys()

    def delete_constrain(self, derivative_order):
        return self.Constraints.pop(derivative_order)

    def get_constrain_orders(self):
        return self.Constraints.keys()

    def get_constrain(self, derivative_order):
        if self.has_constrain(derivative_order):
            return self.Constraints[derivative_order]
        else:
            # print "Vertex ", self.get_index(), "does not have the constrain on ", derivative_order
            return None

    def makeStartOrEnd(self, position, up_to_order):
        position_arr = np.array([position[i] for i in range(self.dimension)])
        self.add_constrain(derivative_order=0, value=position_arr)
        for i in range(1, up_to_order+1):
            value = np.array([0.0 for _ in range(self.dimension)])
            self.add_constrain(derivative_order=i, value=value)


class Constrain(object):
    def __init__(self, order=0, value=0, dimension=1, vertex_indx=0):
        self.constrain = dict()
        self.constrain[order] = value
        self.order = order
        self.dimension = dimension
        self.value = value
        self.vertex_idx = vertex_indx

    def get_order(self):
        return self.order

    def get_constrain_value(self):
        return self.value

    def set_constrain(self, order, value):
        self.order = order
        self.value = value
        self.constrain[order] = value
        self.dimension = len(value)

    def set_vertex_idx(self, index):
        self.vertex_idx = index

    def get_vertex_indx(self):
        return self.vertex_idx

    def get_dimension(self):
        return self.dimension

    def equal(self, constrain):
        return self.order == constrain.get_order()\
               and np.array_equal(self.value, constrain.get_constrain_value())\
               and self.vertex_idx == constrain.get_vertex_indx()\
               and self.dimension == constrain.get_dimension()


class ConstrainData(object):
    def __init__(self, derivative, constrain_value, dimension=1):
        self.order = derivative
        self.value = constrain_value
        self.dimension = dimension

    def get_order(self):
        aa = 0
        return self.order

    def get_value(self):
        return self.value

    def set_constrain(self, order, value):
        self.order = order
        self.value = value


if __name__ == '__main__':
    vertex = Vertex(3)
    vertex.add_constrain(0, 1)
