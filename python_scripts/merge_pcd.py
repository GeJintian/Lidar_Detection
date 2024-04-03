import numpy as np
import open3d as o3d
from tqdm import tqdm

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

def create_transformation_matrix(rpy, xyz):
    rotation_matrix = rpy_to_rotation_matrix(*rpy)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = xyz
    return transformation_matrix

def apply_transformation(pcd, transformation_matrix):
    pcd.transform(transformation_matrix)
    return pcd



if __name__=='__main__':
    left_root = '../data/03_21/lidar_left/'
    right_root = '../data/03_21/lidar_right/'
    front_root = '../data/03_21/lidar_front/'
    save_root = '../data/03_21/merged/'
    for i in tqdm(range(1917)):
        pcd_left = o3d.io.read_point_cloud(left_root+str(i).zfill(9)+".pcd")
        pcd_right = o3d.io.read_point_cloud(right_root+str(i).zfill(9)+".pcd")
        pcd_front = o3d.io.read_point_cloud(front_root+str(i).zfill(9)+".pcd")

        # 定义两个激光雷达的RPY和XYZ变换信息
        lidar_left_rpy = [2.143873331795305, -0.003605912968916103, -1.5671401905346867]
        lidar_left_xyz = [0.05473934211418374, -0.2372464311776421, -0.6578870124268912]

        lidar_right_rpy = [-2.143873331795305, -0.003605912968916103, -1.5744524630551062]
        lidar_right_xyz = [0.16838809275080502, -0.2372464311776421, -0.6578870124268912]

        lidar_front_xyz = [0.11156371743249438, -0.015148333508697098, -0.10643051836532358]
        lidar_front_rpy = [0.048645729083959824, 0.031505277472651416, -1.5728420649351478]

        # 创建变换矩阵
        transformation_matrix_left = create_transformation_matrix(lidar_left_rpy, lidar_left_xyz)
        transformation_matrix_right = create_transformation_matrix(lidar_right_rpy, lidar_right_xyz)
        transformation_matrix_front = create_transformation_matrix(lidar_front_rpy, lidar_front_xyz)

        # 应用变换
        transformed_pcd_left = apply_transformation(pcd_left, transformation_matrix_left)
        transformed_pcd_right = apply_transformation(pcd_right, transformation_matrix_right)
        transformed_pcd_front = apply_transformation(pcd_front, transformation_matrix_front)
        merged_pcd = transformed_pcd_left + transformed_pcd_right + transformed_pcd_front
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
        # o3d.visualization.draw_geometries([merged_pcd,coordinate_frame])
        o3d.io.write_point_cloud(save_root+str(i).zfill(9)+".pcd", merged_pcd)