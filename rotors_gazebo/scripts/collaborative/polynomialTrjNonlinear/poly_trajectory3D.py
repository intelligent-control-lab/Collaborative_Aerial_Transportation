from poly_trajectory import PolyTrajectory
import numpy as np


class PolyTrajectory3D(object):
    def __init__(self, num_segs=1, orders=[12], seg_endT=[3.0], offset=np.zeros((1, 3))):
        self.poly3d_trj = []
        self.poly3d_vel = []
        self.poly3d_acc = []
        self.offset = offset
        self.offset_added = False
        self.poly_x = PolyTrajectory(
            num_segments=num_segs,
            orders=orders,
            seg_endTimes=seg_endT
        )

        self.poly_y = PolyTrajectory(
            num_segments=num_segs,
            orders=orders,
            seg_endTimes=seg_endT
        )

        self.poly_z = PolyTrajectory(
            num_segments=num_segs,
            orders=orders,
            seg_endTimes=seg_endT
        )

    def set_x_params(self, num_segs=None, orders=None, endTs=None):
        if num_segs is not None:
            self.poly_x.set_num_segs(num_seg=num_segs)
        if orders is not None:
            self.poly_x.set_orders(orders=orders)
        if endTs is not None:
            self.poly_x.set_segTs(Ts=endTs)

    def set_y_params(self, num_segs=None, orders=None, endTs=None):
        if num_segs is not None:
            self.poly_y.set_num_segs(num_seg=num_segs)
        if orders is not None:
            self.poly_y.set_orders(orders=orders)
        if endTs is not None:
            self.poly_y.set_segTs(Ts=endTs)

    def set_z_params(self, num_segs=None, orders=None, endTs=None):
        if num_segs is not None:
            self.poly_z.set_num_segs(num_seg=num_segs)
        if orders is not None:
            self.poly_z.set_orders(orders=orders)
        if endTs is not None:
            self.poly_z.set_segTs(Ts=endTs)

    def set_segment_time(self, segment_times):
        """
        :param segment_time: the estimated time (same for x,y and z) used for finishing the segments
                            shape of inputs should be [number_of_segments, 1]
        """
        self.poly_x.set_segTs(segment_times)
        self.poly_y.set_segTs(segment_times)
        self.poly_z.set_segTs(segment_times)

    def solve_traj_xyz(self, sample_frequency):
        self.poly_x.solve()
        self.poly_y.solve()
        self.poly_z.solve()

        trj_x = self.poly_x.get_trajectory(sample_frequency=sample_frequency)
        trj_y = self.poly_y.get_trajectory(sample_frequency=sample_frequency)
        trj_z = self.poly_z.get_trajectory(sample_frequency=sample_frequency)

        trj_vx = self.poly_x.get_d_trajectory(order=1, sample_frequency=sample_frequency)
        trj_vy = self.poly_y.get_d_trajectory(order=1, sample_frequency=sample_frequency)
        trj_vz = self.poly_z.get_d_trajectory(order=1, sample_frequency=sample_frequency)

        trj_accx = self.poly_x.get_d_trajectory(order=2, sample_frequency=sample_frequency)
        trj_accy = self.poly_y.get_d_trajectory(order=2, sample_frequency=sample_frequency)
        trj_accz = self.poly_z.get_d_trajectory(order=2, sample_frequency=sample_frequency)

        # shape: (num_segments * num_intervals, 3)
        self.poly3d_trj = np.concatenate(
            (trj_x, trj_y, trj_z),
            axis=1
        )

        self.poly3d_vel = np.concatenate(
            (trj_vx, trj_vy, trj_vz),
            axis=1
        )

        self.poly3d_acc = np.concatenate(
            (trj_accx, trj_accy, trj_accz),
            axis=1
        )

    def set_offset(self, x, y, z):
        self.offset = np.array([x, y, z])

    def add_offset(self):
        self.poly3d_trj = self.poly3d_trj + self.offset
        self.poly_x.poly_trajectory = self.poly_x.poly_trajectory + self.offset[0]
        self.poly_y.poly_trajectory = self.poly_y.poly_trajectory + self.offset[0]
        self.poly_z.poly_trajectory = self.poly_z.poly_trajectory + self.offset[0]
        self.offset_added = True

    def get_3d_trj(self):
        return self.poly3d_trj

    def get_3d_velocities(self):
        return self.poly3d_vel

    def get_3d_accelerations(self):
        return self.poly3d_acc

    def plot_x(self):
        self.poly_x.plot_poly()
        self.poly_x.plot_derivatives(order=1)
        self.poly_x.plot_derivatives(order=2)

    def plot_y(self):
        self.poly_y.plot_poly()
        self.poly_y.plot_derivatives(order=1)
        self.poly_y.plot_derivatives(order=2)

    def plot_z(self):
        self.poly_z.plot_poly()
        self.poly_z.plot_derivatives(order=1)
        self.poly_z.plot_derivatives(order=2)

    def set_trj_penalties_mode(self, mode='snap'):
        self.poly_x.set_trj_penalties(mode=mode)
        self.poly_y.set_trj_penalties(mode=mode)
        self.poly_z.set_trj_penalties(mode=mode)


if __name__ == '__main__':
    poly3d = PolyTrajectory3D(offset=np.array([0, 0, 0]))
    poly3d.solve_traj_xyz(sample_frequency=50)
    poly3d.add_offset()
    poly3d.plot_x()
    poly3d.plot_y()
    poly3d.plot_z()
