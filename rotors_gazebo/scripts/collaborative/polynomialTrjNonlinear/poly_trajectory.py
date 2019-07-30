from poly_segment import PolySegment
import numpy as np
from matplotlib import pyplot as plt


class PolyTrajectory(object):
    def __init__(self, num_segments, orders, seg_endTimes):
        self.seg_orders = orders
        self.num_segs = num_segments
        self.poly_trajectory = []
        self.poly_velocity = []
        self.poly_acc = []
        self.poly_jerk = []
        self.poly_snap = []
        self.segments = []
        self.seg_length = []
        self.seg_T = []
        self.constrain_orders = []
        self.constrain_zeroValues = []
        self.constrain_endValues = []

        self.t = []
        self.initiate_segments(orders=orders, seg_Ts=seg_endTimes)

        # default values:
        constrain_orders = [
            [0, 1, 2, 3],
            [0, 1, 2, 3],
            [0, 1, 2, 3],
            [0, 1, 2, 3]
        ]
        constrain_zeroValues = [
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 0.5, 0.1, 0.0],
            [2.0, 0.8, 0.2, 0.0],
            [3.0, 1.2, 0.3, 0.0],
        ]
        constrain_endValues = [
            [1.0, 0.5, 0.1, 0.0],
            [2.0, 0.8, 0.2, 0.0],
            [3.0, 1.2, 0.3, 0.0],
            [4.0, 2.4, 0.0, 0.0],
        ]
        self.set_constrains(
            constrain_orders,
            constrain_zeroValues,
            constrain_endValues
        )

    def initiate_segments(self, orders, seg_Ts):
        for i in range(self.num_segs):
            seg = PolySegment(orders[i])
            seg.set_penalties(
                (np.ones((1, seg.variable_length)) / seg.variable_length)[0, :].tolist()
            )
            self.segments.append(seg)

        self.set_orders(orders=orders)
        self.set_segTs(Ts=seg_Ts)
        self.poly_trajectory = [np.zeros((self.seg_length[i], 1)) for i in range(self.num_segs)]

    def set_constrains(self, seg_constrain_orders, seg_zeroValues, seg_endValues):
        for i, seg in enumerate(self.segments):
            seg.set_constrains(
                orders=seg_constrain_orders[i],
                zero_values=seg_zeroValues[i],
                endtime_values=seg_endValues[i]
            )

    def set_num_segs(self, num_seg):
        self.num_segs = num_seg

    def set_penalties(self, penalties):
        for i, seg in enumerate(self.segments):
            seg.set_penalties(penalties[i])

    def set_orders(self, orders):
        if not len(self.seg_orders) == 0:
            self.seg_orders = []
        for i in range(self.num_segs):
            self.seg_length.append(orders[i] + 1)

    def set_segTs(self, Ts):
        for i, seg in enumerate(self.segments):
            seg.set_endTime(endtime=Ts[i])

    def solve(self):
        for seg in self.segments:
            seg.set_initial_values()
            seg.solve()

    def get_trajectory(self, sample_frequency):
        total_samples = 0
        for i, seg in enumerate(self.segments):
            num_intervals = int(sample_frequency * seg.get_endTime())
            seg.sample(num_intervals)
            if i == 0:
                trj_value = seg.samples
            else:
                trj_value = np.concatenate(
                    (trj_value, seg.samples)
                )
            total_samples += num_intervals
        trj_value = trj_value.reshape((total_samples, 1))
        self.poly_trajectory = trj_value
        return trj_value

    def get_d_trajectory(self, order, sample_frequency):
        total_samples = 0
        for i, seg in enumerate(self.segments):
            num_intervals = sample_frequency * seg.get_endTime()
            seg.sample_derivative_value(order=order, num_intevals=num_intervals)
            self.segments[i].sample_derivative_value(order=order, num_intevals=num_intervals)
            if i == 0:
                trj_value = self.segments[i].samples_derivatives
            else:
                trj_value = np.concatenate(
                    (trj_value, self.segments[i].samples_derivatives)
                )
            total_samples += num_intervals
        trj_value = trj_value.reshape((total_samples, 1))
        if order == 1:
            self.poly_velocity = trj_value
        elif order == 2:
            self.poly_acc = trj_value
        elif order == 3:
            self.poly_jerk = trj_value
        elif order == 4:
            self.poly_snap = trj_value
        return trj_value

    def plot_poly(self):
        plt.plot(self.poly_trajectory)
        plt.grid(True)
        plt.show()

    def set_trj_penalties(self, mode='snap'):
        for i, seg in enumerate(self.segments):
            seg.set_objectives_penalty(mode=mode)

    def plot_derivatives(self, order):
        if order == 1:
            plt.plot(self.poly_velocity)
            plt.title('velocity')
        elif order == 2:
            plt.plot(self.poly_acc)
            plt.title('acceleration')
        elif order == 3:
            plt.plot(self.poly_jerk)
            plt.title('jerk')
        elif order == 4:
            plt.plot(self.poly_snap)
            plt.title('snap')
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    num_segs = 2
    poly_orders = [10, 10]
    poly_trj = PolyTrajectory(
                            num_segments=num_segs,
                            orders=poly_orders,
                            seg_endTimes=[4.0, 4.0]
                        )
    poly_trj.set_trj_penalties(mode='snap')
    # cons_orders = [
    #     [0, 1, 2, 3, 4],
    #     [0, 1, 2, 3, 4]
    # ]
    # cons_zeroValues = [
    #     [0.0, 0.0, 0.0, 0.0, 0.0],
    #     [3.0, 0.0, 0.0, 0.0, 0.0]
    # ]
    # cons_endValues = [
    #     [3.0, 0.0, 0.0, 0.0, 0.0],
    #     [6.0, 0.0, 0.0, 0.0, 0.0]
    # ]
    # poly_trj.set_constrains(
    #                         cons_orders,
    #                         cons_zeroValues,
    #                         cons_endValues
    #                     )

    constrain_orders = [
        [0, 1, 2, 3, 4],
        [0, 1, 2, 3, 4]

    ]
    constrain_zeroValues = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 0.0]
    ]
    constrain_endValues = [
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0, 0.0, 0.0]
    ]
    poly_trj.set_constrains(
        constrain_orders,
        constrain_zeroValues,
        constrain_endValues
    )


    poly_trj.solve()
    poly_trj.get_trajectory(sample_frequency=50)
    poly_trj.plot_poly()
    poly_trj.get_d_trajectory(order=1, sample_frequency=50)
    poly_trj.get_d_trajectory(order=2, sample_frequency=50)
    poly_trj.get_d_trajectory(order=3, sample_frequency=50)
    poly_trj.get_d_trajectory(order=4, sample_frequency=50)
    poly_trj.plot_derivatives(order=1)
    poly_trj.plot_derivatives(order=2)
    poly_trj.plot_derivatives(order=3)
    poly_trj.plot_derivatives(order=4)
