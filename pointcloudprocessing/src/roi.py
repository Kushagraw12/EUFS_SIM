#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import pcl_helper

def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)

    filter_axis = 'x'
    axis_min = 0.0
    axis_max = 20.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -5.0
    axis_max = 5.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    cloud_new = pcl_helper.pcl_to_ros(cloud)
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node("roi", anonymous=True)
    rospy.Subscriber('/velodyne', PointCloud2, callback)
    pub = rospy.Publisher("/velodyne_points_roi", PointCloud2, queue_size=1)
    rospy.spin()