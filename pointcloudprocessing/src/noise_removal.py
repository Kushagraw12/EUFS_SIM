#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
 
import pcl
import pcl_helper



 
def do_statistical_outlier_filtering(cloud , mean_k , thresh):
 outlier_filter = cloud.make_statistical_outlier_filter()
 outlier_filter.set_mean_k(mean_k)
 outlier_filter.set_std_dev_mul_thresh(thresh)
 return outlier_filter.filter()
 
def callback(input_ros_msg):
 cloud = pcl_helper.ros_to_pcl(input_ros_msg)
 print("INPUT: ", cloud, type(cloud))
 cloud = pcl_helper.XYZRGB_to_XYZ(cloud)
 mean_k = 10
 thresh = 0.001
 cloud = do_statistical_outlier_filtering(cloud , mean_k , thresh)
 color = pcl_helper.random_color_gen()
 cloud = pcl_helper.XYZ_to_XYZRGB(cloud , color)
 cloud_new = pcl_helper.pcl_to_ros(cloud)
 print("OUTPUT: ", cloud, type(cloud))
 pub.publish(cloud_new)



 
if __name__ == "__main__":
 rospy.init_node("noise_removal" , anonymous= True)
 
 rospy.Subscriber("/velodyne" , PointCloud2 , callback)
 pub = rospy.Publisher("/velodyne_new" , PointCloud2 , queue_size=1)
 rospy.spin()