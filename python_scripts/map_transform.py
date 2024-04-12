import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    
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

pcd_rtk = o3d.io.read_point_cloud("/home/oscar/Downloads/dlo_map.pcd")
pcd_yas = o3d.io.read_point_cloud("/home/oscar/Downloads/yas_map.pcd")

rtk_down = pcd_rtk.voxel_down_sample(1)
yas_down = pcd_yas.voxel_down_sample(1)

xyz=[-30, -100, 5] 
rpy=[0, 0, 0]
left_T = create_transformation_matrix(rpy,xyz)
print("before",left_T)

#result = o3d.pipelines.registration.registration_generalized_icp(pcd_rtk, pcd_yas,10,left_T)
#result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(pcd_rtk, pcd_yas,2,left_T)

#print("result",result.transformation)
source_temp = copy.deepcopy(pcd_yas)
target_temp = copy.deepcopy(pcd_rtk)
source_temp.paint_uniform_color([1, 0.706, 0])
target_temp.paint_uniform_color([0, 0.651, 0.929])
source_temp.transform(left_T)
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0,0,0])
o3d.visualization.draw_geometries([source_temp, target_temp,coordinate_frame])

#draw_registration_result(pcd_yas, pcd_rtk, result.transformation)