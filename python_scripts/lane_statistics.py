import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

def filtered_pcd(file_path):
    with open(file_path, 'r') as file:
        in_header = True
        pcd = []
        for line in file:
            # 跳过文件头
            if in_header:
                if line.startswith('DATA'):
                    in_header = False
                continue
            # 读取点云数据
            parts = line.strip().split()
            if len(parts) == 4:  # 确保是POINT_XYZI格式
                # 提取intensity并添加到列表
                if(abs(float(parts[3])-160)<=20):
                    intensity = [float(parts[0]),float(parts[1]),float(parts[2])]
                    pcd.append(intensity)
    return pcd

def read_pcd(file_path):
    """读取PCD文件并返回intensity列表"""
    with open(file_path, 'r') as file:
        in_header = True
        intensities = []
        for line in file:
            # 跳过文件头
            if in_header:
                if line.startswith('DATA'):
                    in_header = False
                continue
            # 读取点云数据
            parts = line.strip().split()
            if len(parts) == 4:  # 确保是POINT_XYZI格式
                # 提取intensity并添加到列表
                if(abs(float(parts[2]))<=15):
                    intensity = float(parts[3])
                    intensities.append(intensity)
    return intensities

def plot_histogram(intensities, bins=50):
    """绘制intensity的柱状图"""
    plt.hist(intensities, bins=bins)
    plt.title('Point Cloud Intensity Distribution')
    plt.xlabel('Intensity')
    plt.ylabel('Frequency')
    plt.show()

if __name__=="__main__":
    # 修改为你的PCD文件路径
    pcd_file_path = '../data/03_21/lidar_right/000000000.pcd'
    #intensities = read_pcd(pcd_file_path)
    # 读取intensity数据
    # pcd = o3d.io.read_point_cloud(pcd_file_path)

    # points = np.asarray(pcd.points)
    # filtered_points = points[abs(points[:,2])<=15]
    
    filtered_points = np.array(filtered_pcd(pcd_file_path))
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    o3d.visualization.draw_geometries([filtered_pcd])
    # 绘制柱状图
    #plot_histogram(intensities)