import numpy as np
import tf
import math


def deplacement_moment_inertia(deplacement, old_inertia, mass):
    x_bar = deplacement[0]
    y_bar = deplacement[1]
    z_bar = deplacement[2]

    new_inertia = np.zeros((3, 3))
    new_inertia[0, 0] = old_inertia[0, 0] + mass * (y_bar * y_bar + z_bar * z_bar)
    new_inertia[1, 1] = old_inertia[1, 1] + mass * (x_bar * x_bar + z_bar * z_bar)
    new_inertia[2, 2] = old_inertia[2, 2] + mass * (x_bar * x_bar + y_bar * y_bar)
    new_inertia[0, 1] = old_inertia[0, 1] - mass * x_bar * y_bar
    new_inertia[1, 0] = old_inertia[0, 1] - mass * x_bar * y_bar
    new_inertia[0, 2] = old_inertia[0, 2] - mass * x_bar * z_bar
    new_inertia[2, 0] = old_inertia[0, 2] - mass * x_bar * z_bar
    new_inertia[1, 2] = old_inertia[1, 2] - mass * y_bar * z_bar
    new_inertia[2, 1] = old_inertia[1, 2] - mass * y_bar * z_bar

    return new_inertia

def skewsymetric(input_vector):
    output_matrix = np.zeros((3, 3))
    output_matrix[0, 0] = 0
    output_matrix[1, 1] = 0
    output_matrix[2, 2] = 0
    output_matrix[0, 1] = -input_vector[2]
    output_matrix[0, 2] = input_vector[1]
    output_matrix[1, 2] = -input_vector[0]
    output_matrix[1, 0] = -output_matrix[0, 1]
    output_matrix[2, 0] = -output_matrix[0, 2]
    output_matrix[2, 1] = -output_matrix[1, 2]
    return output_matrix

def box_inertia(x, y, z, mass):
    box_inertia = np.zeros((3, 3))
    box_inertia[0, 0] = 0.0833333 * mass * (y * y + z * z)
    box_inertia[1, 1] = 0.0833333 * mass * (x * x + z * z)
    box_inertia[2, 2] = 0.0833333 * mass * (x * x + y * y)
    return box_inertia

def inertial_dict2matrix(inertial_dict):
    inertial = np.zeros((3, 3))
    inertial[0, 0] = inertial_dict['ixx']
    inertial[0, 1] = inertial_dict['ixy']
    inertial[0, 2] = inertial_dict['ixz']
    inertial[1, 1] = inertial_dict['iyy']
    inertial[1, 2] = inertial_dict['iyz']
    inertial[2, 2] = inertial_dict['izz']
    inertial[1, 0] = inertial[0, 1]
    inertial[2, 0] = inertial[0, 2]
    inertial[2, 1] = inertial[1, 2]
    return inertial

def vee(Matrix):
    return np.array([
        [Matrix[2, 1]],
        [Matrix[0, 2]],
        [Matrix[1, 0]]
    ]).reshape((3, 1))

def quaternion2euler(data_odom):
    return tf.transformations.euler_from_quaternion([data_odom.pose.pose.orientation.x,
                                                     data_odom.pose.pose.orientation.y,
                                                     data_odom.pose.pose.orientation.z,
                                                     data_odom.pose.pose.orientation.w]
                                                    )

def rotation_matrix_from_quaternion(quat):
    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,         1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,      1 - 2*qx*qx - 2*qy*qy]
    ])

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])