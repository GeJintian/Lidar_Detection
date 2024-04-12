import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from a2rl_bs_msgs.msg import VectornavIns
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import math

def pointcloud2_to_array(msg):
    # 生成器，将PointCloud2数据转换为点列表
    gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z","intensity"))

    # 将生成器转换为列表，然后转换为NumPy数组
    points = np.array(list(gen))

    return points

class RTKMapper(Node):
    def __init__(self):
        super().__init__('rtk_mapper')

        # Initialize subscribers
        self.cloud_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.pose_sub = self.create_subscription(VectornavIns, '/a2rl/vn/ins', self.pose_callback, 10)

        # Initialize point cloud map
        self.point_cloud_map = o3d.geometry.PointCloud()
        # Initialize buffer for storing pose messages
        self.ins_buffer = []
        self.count = 0
        self.count_last = 0
        self.timer = self.create_timer(10, self.check_count)
        self.last_pose = (0,0)
        self.last_yaw = 0
        self.z = -27.24746895
    def get_distance(self,pose):
        result = math.sqrt((self.last_pose[0]-pose[0])**2+(self.last_pose[1]-pose[1])**2)
        return result

    def get_rotate(self,yaw):
        result = abs(yaw-self.last_yaw)
        return result

    def cloud_callback(self, cloud_msg):
        # Process point cloud message

        # Check if there are pose messages in the buffer
        if self.count % 20 == 0:
            if len(self.ins_buffer) == 0:
                self.get_logger().warn("No pose messages received yet. Skipping point cloud processing.")
                return

            # Find the closest pose message based on timestamp
            # closest_pose_msg = min(self.ins_buffer, key=lambda pose_msg: abs(pose_msg.timestamp.nanoseconds * 1e-9 -  cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec*1e-9))
            closest_pose_msg = self.ins_buffer[-1]
            pose = (closest_pose_msg.position_enu_ins.x,closest_pose_msg.position_enu_ins.y)
            # Process synchronized point cloud and pose data

            # Accumulate transformed points to point_cloud_map
            if self.get_distance(pose) > 100 or self.get_rotate(closest_pose_msg.orientation_ypr.z)>0.8:
                transformed_cloud = self.transform_point_cloud(cloud_msg, closest_pose_msg)
                self.last_pose = pose
                self.last_yaw = closest_pose_msg.orientation_ypr.z
                self.point_cloud_map += transformed_cloud
            # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0,0,0])
            # o3d.visualization.draw_geometries([self.point_cloud_map,coordinate_frame])
        self.count += 1

    def pose_callback(self, ins_msg):
        # Process pose message
        # print("receiving pose info")
        # Append pose message to buffer
        self.ins_buffer.append(ins_msg)

    def transform_point_cloud(self, cloud_msg, ins_msg):
        # Extract yaw, pitch, and roll angles from the pose message
        yaw = ins_msg.orientation_ypr.z
        pitch = ins_msg.orientation_ypr.y
        roll = ins_msg.orientation_ypr.x

        # Calculate rotation matrix from yaw, pitch, and roll angles
        r = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        rotation_matrix = r.as_matrix()

        # Extract translation vector from the pose message
        translation_vector = np.array([
            ins_msg.position_enu_ins.x,
            ins_msg.position_enu_ins.y,
            self.z
        ])

        # Create a homogeneous transformation matrix
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        homogeneous_matrix[:3, 3] = translation_vector

        # Extract field names and offsets from the PointCloud2 message
        fields = {field.name: field.offset for field in cloud_msg.fields}

        # Extract XYZ coordinates and intensity data from the PointCloud2 message
        points = np.frombuffer(cloud_msg.data, dtype=np.float32).reshape(-1, cloud_msg.point_step // 4)
        #points = pointcloud2_to_array(cloud_msg)
        #xyz = points[:, [fields['x'], fields['y'], fields['z']]]
        #intensity = points[:, fields['intensity']]

        # Convert cloud_msg to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[points[:,0]**2+points[:,1]**2<10000, :3])
        downpcd = pcd.voxel_down_sample(1)
        print(translation_vector)

        # Transform point cloud based on pose
        transformed_cloud = downpcd.transform(homogeneous_matrix)
        #print("Transform works!")
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0,0,0])
        # o3d.visualization.draw_geometries([transformed_cloud,coordinate_frame])

        return transformed_cloud

    def save_point_cloud_map(self, filename):
        if len(self.point_cloud_map.points) == 0:
            print("Point cloud map is empty. Skipping saving.")
            return
        o3d.io.write_point_cloud(filename, self.point_cloud_map)

    def run(self):
        rclpy.spin(self)

    def check_count(self):
        if self.count == self.count_last:
            self.stop_node()
        self.count_last = self.count

    def stop_node(self):
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    mapper = RTKMapper()
    try:
        mapper.run()
    finally:
        # Save the point cloud map before shutting down
        mapper.save_point_cloud_map("point_cloud_map.pcd")

if __name__ == '__main__':
    main()