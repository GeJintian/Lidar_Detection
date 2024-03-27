import os
import sys
from sklearn.cluster import DBSCAN
import math
import open3d
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image, ImageDraw


def build_bev_points(x,z,result):
    """
    Input:
        x: float; x coordinate in 3D
        z: float; z coordinate in 3D
        result: (n,2) array; contains all bev points
    Effect:
        Transfer a voxel in 3D coordinate system into a bev point
    """
    y_axis = math.floor(z/0.1)
    #print(id1)
    x_axis = math.floor(x/0.1)
    result[159+x_axis][y_axis] += 1

def add_bbx(point_set, is_pred = True):
    """
    Input:
        point_set: (8,3) array; with the order of 
                  5 -------- 6
                 /|         /|
                1 -------- 2 .
                | |        | |
                . 4 -------- 7
                |/         |/
                0 -------- 3
        is_pred: bool; if this bouding box is predicted or ground truth (maybe helpful if you want to visualize both prediction and ground truth)
    Output:
        bbx: LineSet format in open3d
    Effect:
        This function will convert 8 corner points into lineset which open3d could draw
    """
    # point_set should be 8 points
    lines = [[0, 1], [1, 2], [2, 3], [0, 3],
            [4, 5], [5, 6], [6, 7], [4, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    if is_pred:
        colors = [[1, 0, 0] for _ in range(len(lines))]
    else:
        colors = [[0, 1, 0] for _ in range(len(lines))]
    bbx = open3d.open3d.geometry.LineSet()
    bbx.points = open3d.open3d.utility.Vector3dVector(point_set)
    bbx.lines = open3d.open3d.utility.Vector2iVector(lines)
    bbx.colors = open3d.open3d.utility.Vector3dVector(colors)
    return bbx

def use_dbscan(pcd):
## TODO: SETTINGS
    points = pcd.points
    pcd.paint_uniform_color([0,0,1])
    color = np.array(pcd.colors)
    neps = 0.5
    minsp = 100
    clustering = DBSCAN(eps=neps, min_samples = minsp).fit(points)
    counting = 0
    num = 0
    #print(max(clustering.labels_))
    for i in range(np.max(clustering.labels_)+1):
        color[clustering.labels_==i]=[i/np.max(clustering.labels_),1-i/np.max(clustering.labels_),0]
        # if counting < clustering.labels_[ clustering.labels_ == i ].size:
        #     counting = clustering.labels_[ clustering.labels_ == i ].size
        #     num = i
    pcd.colors = open3d.utility.Vector3dVector(color[:,:3])
    open3d.visualization.draw_geometries([pcd])
    # print("num",num)
    # print("len",i)
    # print("counting",counting)
    # print(n_pc_list.shape)
    # print(n_pc_list[clustering.labels_ == num].shape)
    # print(n_pc_list[clustering.labels_ == num][:][2].shape)
    # sys.exit()
    #print(i)
    # Find the cluster with most points. That will be our target object.
    # if max(clustering.labels_) > -1:
    #     print("DBSCAN works")
    #     zmin_T = np.min(pc_list[clustering.labels_ == num].T[2])
    #     zmax = np.max(pc_list[clustering.labels_ == num].T[2])
    #     depth_statistic = pc_list[clustering.labels_ == num].T[2]
    #     sigma = np.std(depth_statistic)
    #     miu = np.mean(depth_statistic)
    #     pc_list = pc_list[clustering.labels_ == num]

def read_pcd(file_path):
    pcd = open3d.io.read_point_cloud(file_path)
    #points = np.asarray(pcd.points)
    return pcd

def main(file):
    pcd = read_pcd(file)
    #open3d.visualization.draw_geometries([pcd])
    use_dbscan(pcd)
    

if __name__ == "__main__":
    file_name = "../data/000001000.pcd"
    main(file_name)