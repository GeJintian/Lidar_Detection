import numpy as np
from matplotlib import pyplot as plt

from sklearn.linear_model import RANSACRegressor
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import PolynomialFeatures
from sklearn.neighbors import KDTree

import copy
import open3d as o3d


def rpy_to_rotation_matrix(roll, pitch, yaw):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    return Rz_yaw @ Ry_pitch @ Rx_roll

def rotation_matrix_to_euler_angles(R):
    assert(R.shape == (3, 3))

    # 计算俯仰角 Pitch
    if np.isclose(R[2, 0], -1):
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(R[0, 1], R[0, 2])
    elif np.isclose(R[2, 0], 1):
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-R[0, 1], -R[0, 2])
    else:
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))

    return roll, pitch, yaw

def create_transformation_matrix(rpy, xyz):
    rotation_matrix = rpy_to_rotation_matrix(*rpy)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = xyz
    return transformation_matrix

class point():
    def __init__(self, translation, rotation):
        self.translation = translation
        self.roation = rotation
        self.transform = np.concatenate((np.concatenate((rotation,np.array([translation]).transpose()),axis=1),np.array([[0,0,0,1]])),axis=0)

def read_traj(file_name):
    point_set = []
    points_array = []
    f = open(file_name, 'r')
    for line in f:
        parts = line.split()
        x, y, z = float(parts[3]), float(parts[7]), float(parts[11])
        trans = np.array([x, y, z])
        points_array.append(trans)
        r0, r1, r2, r3, r4, r5, r6, r7, r8 = map(float, parts[:3] + parts[4:7] + parts[8:11])
        rotation_matrix = [[r0, r1, r2], [r3, r4, r5], [r6, r7, r8]]
        rotat=np.array(rotation_matrix)
        
        point_set.append(point(trans,rotat))
    f.close()
    return point_set, points_array

def read_traj_wo_z(file_name):
    point_set = []
    points_array = []
    f = open(file_name, 'r')
    for line in f:
        parts = line.split(',')
        x, y = float(parts[0]), float(parts[1])
        trans = np.array([x, y, 0])
        points_array.append(trans)
    f.close()
    return point_set, points_array

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

# left_traj_file = "/home/oscar/workspace/a2rl/automan-a2rl/src/localization/src/utils/left.txt"
# right_traj_file = "/home/oscar/workspace/a2rl/automan-a2rl/src/localization/src/utils/right.txt"
# front_traj_file = "/home/oscar/workspace/a2rl/automan-a2rl/src/localization/src/utils/front.txt"

# front_xyz=[0.11156371743249438, -0.015148333508697098, -0.10643051836532358] 
# front_rpy=[0.048645729083959824, 0.031505277472651416, -1.5728420649351478]

# left_xyz=[0.05473934211418374, -0.2372464311776421, -0.6578870124268912] 
# left_rpy=[2.143873331795305, -0.003605912968916103, -1.5671401905346867]

# right_xyz=[0.16838809275080502, -0.2372464311776421, -0.6578870124268912] 
# right_rpy=[-2.143873331795305, -0.003605912968916103, -1.5744524630551062]

# front_T = create_transformation_matrix(front_rpy,front_xyz)
# left_T = create_transformation_matrix(left_rpy,left_xyz)
# right_T = create_transformation_matrix(right_rpy,right_xyz)

# l_p, l_a = read_traj(left_traj_file)
# r_p, r_a = read_traj(right_traj_file)
# f_p, f_a = read_traj(front_traj_file)

# pcd_left = o3d.geometry.PointCloud()
# pcd_left.points = o3d.utility.Vector3dVector(np.array(l_a))

# pcd_right = o3d.geometry.PointCloud()
# pcd_right.points = o3d.utility.Vector3dVector(np.array(r_a))

# pcd_front = o3d.geometry.PointCloud()
# pcd_front.points = o3d.utility.Vector3dVector(np.array(f_a))

lidar_traj_file = "/home/oscar/workspace/a2rl/dataset/rtk_path/gicp_poses.csv"
rtk_traj_file = "/home/oscar/workspace/a2rl/dataset/rtk_path/rtk_poses.csv"

_, l_a = read_traj_wo_z(lidar_traj_file)
_, r_a = read_traj_wo_z(rtk_traj_file)

pcd_rtk = o3d.geometry.PointCloud()
pcd_rtk.points = o3d.utility.Vector3dVector(np.array(r_a))

pcd_lidar = o3d.geometry.PointCloud()
pcd_lidar.points = o3d.utility.Vector3dVector(np.array(l_a))

init_guess = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
result = o3d.pipelines.registration.registration_generalized_icp(pcd_lidar, pcd_rtk,10, init_guess) #rtk to lidar
roll,pitch,yaw = rotation_matrix_to_euler_angles(np.array(result.transformation)[:3,:3])
#print("init ",np.dot(np.linalg.inv(front_T),right_T))
print("result",result.transformation)
print("right to front: roll ",roll," pitch ",pitch, "yaw", yaw)


source_temp = copy.deepcopy(pcd_rtk)
target_temp = copy.deepcopy(pcd_lidar)
source_temp.paint_uniform_color([1, 0.706, 0])
target_temp.paint_uniform_color([0, 0.651, 0.929])
# source_temp.transform(right_T)
# target_temp.transform(front_T)
#o3d.visualization.draw_geometries([source_temp, target_temp])
draw_registration_result(pcd_lidar, pcd_rtk, result.transformation)


# result = o3d.pipelines.registration.registration_generalized_icp(pcd_left, pcd_front,1, np.dot(np.linalg.inv(front_T),left_T))
# roll,pitch,yaw = rotation_matrix_to_euler_angles(np.array(result.transformation)[:3,:3])
# #print("init ",np.dot(np.linalg.inv(front_T),right_T))
# print("result",result.transformation)
# print("right to front: roll ",roll," pitch ",pitch, "yaw", yaw)