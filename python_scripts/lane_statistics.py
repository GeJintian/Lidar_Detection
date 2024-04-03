import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import math
from scipy.linalg import eigh

from sklearn.cluster import HDBSCAN,DBSCAN,OPTICS
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt

import pypatchworkpp

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
                #if(float(parts[0])>0 and float(parts[0])<3):
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

class bin_container:
    def __init__(self,x,y):
        self.y_num = y
        self.x_num = x
        self.bin_set = [[[] for _ in range(y)] for _ in range(x)]
        self.count = 0
        
    def add_one(self, p, x, y):
        self.bin_set[x][y].append(p)
        self.count += 1

def split_into_bin(points):
    res = 1
    width = 100
    length = 100
    y_num = res*width
    x_num = res*length
    bin_list = bin_container(x_num,y_num)
    print(len(points))
    for p in points:
        if p[2] >=0 and p[2]<length and p[1]>=-width/2 and p[1]<width/2:
            bin_list.add_one(p,math.floor(p[2]*res),math.floor(p[1]*res+y_num/2))
    return bin_list

def filtering(input_points):
    points = np.zeros(input_points.shape)
    points[:,2] = -input_points[:,1]
    points[:,1] = input_points[:,0]
    points[:,0] = input_points[:,2]
    
    length = 100
    width = 80
    new_points = []
    for p in points:
        #if p[2] >=0 and p[2]<length and p[1]>=-width/2 and p[1]<width/2:
        if ((p[2]>0 and 10 > p[0] >0 ) or (p[2] > -1 and (p[0] <=0 or p[0]>=10))) and p[2]<4:
            if p[1]>-30 and p[1]<30 and p[0]>-50 and p[0]<100 and p[1]**2 + p[0]**2>16:
                new_points.append(p)
    return np.array(new_points)

def plotting(points, name):
    #flaten
    points = np.concatenate(points, axis=0)
    
    x_min, x_max = -100, 20
    y_min, y_max = -20, 20
    filtered_points = points[(points[:, 0] > x_min) & (points[:, 0] < x_max) & 
                            (points[:, 1] > y_min) & (points[:, 1] < y_max)]

    plt.figure(figsize=(10, 5))  # 可以调整图像大小
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], c='blue', s=1)  # s控制点的大小
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_facecolor('white')  # 设置背景颜色为白色

    # 保存图像，确保背景色在保存时为白色
    plt.savefig('/home/oscar/workspace/a2rl/Lidar_Detection/data/03_21/2d_plot/'+name+'.png', facecolor='white')
    plt.close()

if __name__=="__main__":
    np.random.seed(42)  # For reproducibility
    MAX_COLOR = np.random.randint(0, 256, size=(300, 3))/255.0

    MAX_COLOR.tolist()
    params = pypatchworkpp.Parameters()
    PatchworkPLUSPLUS = pypatchworkpp.patchworkpp(params)
    params.verbose = False
    #for idx in range(1917):
    # 修改为你的PCD文件路径
    idx = 1
    pcd_file_path = '../data/03_21/merged/'+str(idx).zfill(9)+'.pcd'
    #intensities = read_pcd(pcd_file_path)
    # 读取intensity数据
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    np_points = filtering(np.asarray(pcd.points))
    #points = o3d.utility.Vector3dVector()
    
    PatchworkPLUSPLUS.estimateGround(np_points)
    
    ground      = PatchworkPLUSPLUS.getGround()
    nonground   = PatchworkPLUSPLUS.getNonground()
    
    ground_o3d = o3d.geometry.PointCloud()
    ground_o3d.points = o3d.utility.Vector3dVector(ground)
    ground_o3d.paint_uniform_color([0,0,1])
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(nonground)
    filtered_pcd.paint_uniform_color([0,0,1])

    points = np.asarray(filtered_pcd.points)
    # bin_list = split_into_bin(points)
    color = np.array(filtered_pcd.colors)
    neps = 0.5
    minsp = 5
    clustering = DBSCAN(eps=neps, min_samples = minsp,algorithm='kd_tree').fit(points)
    #print(max(clustering.labels_))
    # for i in range(np.max(clustering.labels_)+1):
    #     color[clustering.labels_==i]=[i/np.max(clustering.labels_),1-i/np.max(clustering.labels_),0]
    
    z_vector = np.array([0,0,1])
    #gm = GaussianMixture(n_components=50, random_state=0).fit(points)
    #clustering = gm.predict(points)
    thres = 1
    # clustering = HDBSCAN(min_cluster_size=10,min_samples = 100).fit(points)
    clusters = []
    clusters_cov = []
    clusters_trace = []
    clusters_values = []
    clusters_for_plt = []
    for i in range(np.max(clustering.labels_)+1):
        clusters.append(points[clustering.labels_==i])
    for i in range(len(clusters)):
        if len(clusters[i]) > 100 and len(clusters[i]) < 1000:
            # cov = np.cov(clusters[i], rowvar=False)
            # clusters_cov.append(cov)
            # eigenvalues, _ = np.linalg.eig(cov)
            # eigenvalues = eigenvalues/np.linalg.norm(eigenvalues)
            # clusters_values.append(eigenvalues)
            # sorted_eigenvalues = np.sort(eigenvalues)[::-1]
            # if sorted_eigenvalues[0] / sorted_eigenvalues[1]>thres:

            cov = np.cov(clusters[i], rowvar=False)
            clusters_cov.append(cov)
            eigenvalues, eigenvectors = eigh(cov)
            normal_vector = eigenvectors[:, np.argmin(eigenvalues)]
            cos_theta = np.dot(normal_vector,z_vector)/(np.linalg.norm(normal_vector)*np.linalg.norm(z_vector))
            if cos_theta<0.5:
                color[clustering.labels_==i]=MAX_COLOR[i]
                clusters_for_plt.append(clusters[i])
        if len(clusters[i]) > 1000:
            color[clustering.labels_==i]=MAX_COLOR[i]
            clusters_for_plt.append(clusters[i])
    # print(clusters_trace)
    # print(clusters_values)

    filtered_pcd.colors = o3d.utility.Vector3dVector(color[:,:3])
    #plotting(clusters_for_plt,str(idx).zfill(9))
    # thres = 0.05
    # ground_list = []
    # for x in range(bin_list.x_num):
    #     for y in range(bin_list.y_num):
    #         if len(bin_list.bin_set[x][y])>0:
    #             point_set = np.array(bin_list.bin_set[x][y])
    #             max_height = np.amax(point_set[:,0])
    #             min_height = np.amin(point_set[:,0])
    #             if max_height-min_height < thres:
    #                 ground_list+=bin_list.bin_set[x][y]
    # print(len(ground_list))
    # filtered_points = np.array(ground_list)
    # print("y_max",np.amax(filtered_points[:,1]))
    # print("y_min",np.amin(filtered_points[:,1]))
    # print("x_max",np.amax(filtered_points[:,2]))
    # print("x_min",np.amin(filtered_points[:,2]))
    # filtered_pcd = o3d.geometry.PointCloud()
    # filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    
    
    # colors = [[1, 0, 0] if 0 <= point[0] <= 3 else [0, 0, 1] for point in filtered_points]
    # filtered_pcd.colors = o3d.utility.Vector3dVector(colors)
    
    #coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    # vis = o3d.visualization.VisualizerWithKeyCallback()
    # vis.create_window(width = 600, height = 400)
    # #vis.add_geometry(ground_o3d)
    # vis.add_geometry(coordinate_frame)
    # vis.add_geometry(filtered_pcd)

    # vis.run()
    # vis.destroy_window()
    
    o3d.visualization.draw_geometries([filtered_pcd])
    # 绘制柱状图
    #plot_histogram(intensities)