#!/usr/bin/env python3

import os
import argparse
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

def get_rosbag_options(path, storage_id, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(
        uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def sequential_reader(bag_path, topic):

    storage_id = rosbag2_py.get_default_storage_id()
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    # Set files names
    f_radar_bin = open(os.path.basename(bag_path)+'.bin', "wb")
    f_radar_txt = open(os.path.basename(bag_path)+'.txt', "w")
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        # Write files
        f_radar_bin.write(msg.raw_data)
        f_radar_txt.write(str(t) + "\n")
    # Close files   
    f_radar_bin.close()
    f_radar_txt.close()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="rosbag2 converter")
    parser.add_argument("bag_path")
    parser.add_argument("topic")
    args = parser.parse_args()
    sequential_reader(args.bag_path, args.topic)
