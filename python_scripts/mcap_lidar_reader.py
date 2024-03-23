import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader

def write_to(output_path: str):
    TOPIC_NAME = '1'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=TOPIC_NAME, type="std_msgs/msg/String", serialization_format="cdr"
        )
    )

    start_time = 0
    for i in range(10):
        msg = String()
        msg.data = f"Chatter #{i}"
        timestamp = start_time + (i * 100)
        writer.write(TOPIC_NAME, serialize_message(msg), timestamp)

    del writer

def parser(input_bag:str, output_bag:str):
    TOPIC1 = '/sensor/lidar_left/points'
    TOPIC2 = '/sensor/lidar_front/points'
    TOPIC3 = '/sensor/lidar_right/points'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=TOPIC1, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=TOPIC2, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=TOPIC3, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"
        )
    )
    
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")
    count = 0
    while reader.has_next() and count < 600*3:
        topic, data, timestamp = reader.read_next()
        if topic == TOPIC1 or topic == TOPIC2 or topic == TOPIC3:
            #print("recording")
            writer.write(topic, data, timestamp)
        
            count = count + 1
        
    del writer
    del reader

if __name__ =="__main__":
    bag_name = '/home/oscar/workspace/a2rl/dataset/2024_03_21/rosbag2_2024_03_21-12_07_43/rosbag2_2024_03_21-12_07_43_0.mcap'
    output_name = '/home/oscar/workspace/a2rl/dataset/2024_03_21_modified/rosbag2_2024_03_21-12_07_43'
    parser(bag_name, output_name)