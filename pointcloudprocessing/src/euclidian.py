#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
 
import pcl
import pcl_helper

def do_euclidian_clustering(white_cloud):
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = pcl_helper.get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            pcl_helper.rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud, cluster_indices


def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    #color = pcl_helper.random_color_gen()
    #white_cloud = pcl_helper.XYZ_to_XYZRGB(cloud, color)
    cluster_cloud, cluster_indices = do_euclidian_clustering(cloud)
    pub.publish(cluster_cloud)


 
if __name__ == "__main__":
 rospy.init_node("euclidian" , anonymous= True)
 
 rospy.Subscriber("/velodyne" , PointCloud2 , callback)
 pub = rospy.Publisher("/velodyne_euclidian" , PointCloud2 , queue_size=1)
 rospy.spin()