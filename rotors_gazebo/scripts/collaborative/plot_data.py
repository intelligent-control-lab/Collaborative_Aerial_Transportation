from matplotlib import pyplot as plt
import numpy as np
import math
import csv
import pandas as pd


if __name__ == '__main__':
    # data order:
    # x, y, z, force_x, force_y, force_z, torque_x, torque_y, torque_z

    csv_file = [
        '/home/lucasyu/gazebo_learning_ws/src/collaborative_rotorS/force_torque_data/hummingbird_0_2019_07_19_11:35:42.csv'
                ]
    data_collection = []
    for i_csv in csv_file:
        with open(i_csv, 'r') as csvfile:
            df = pd.read_csv(i_csv)
            data_collection.append(df.values)

    plt.plot(data_collection[0][:, 3])
    plt.grid(True)
    plt.show()