#!/usr/bin/env python

from pcl_helper import *
from filtering_helper import *

def noise_removal(cloud):
 
 mean_k = 10
 thresh = 0.001
 cloud = do_statistical_outlier_filtering(cloud , mean_k , thresh)
 color = random_color_gen()
 cloud = XYZ_to_XYZRGB(cloud , color)
 cloud_new = pcl_to_ros(cloud)
 return cloud_new


def split_cloud(cloud):

 # cloud_denoised = noise_removal(cloud)

  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)

  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'x', min_axis = 0.0, max_axis = 20.0)
  filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         name_axis = 'y', min_axis = -5.0, max_axis = 5.0)

  table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.15)

  return objects_cloud, table_cloud

def get_clusters(cloud, tolerance, min_size, max_size):

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()

  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)

  clusters = extraction_object.Extract()
  return clusters
  
def get_colored_clusters(clusters, cloud):

  number_of_clusters = len(clusters)
  colors = random_color_gen()

  colored_points = []

  for cluster_id, cluster in enumerate(clusters):
    for c, i in enumerate(cluster):
      x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
      color = rgb_to_float(colors)
      colored_points.append([x, y, z, color])
  
  return colored_points




def pcl_callback(pcl_msg):

   cloud = ros_to_pcl(pcl_msg) 

   objects_cloud, table_cloud = split_cloud(cloud) 

   colorless_cloud = XYZRGB_to_XYZ(objects_cloud)

   clusters = get_clusters(colorless_cloud, tolerance = 0.05, min_size = 20, max_size = 1500)

   colored_points = get_colored_clusters(clusters, colorless_cloud)

   clusters_cloud = pcl.PointCloud_PointXYZRGB()
   clusters_cloud.from_list(colored_points)
   cloud_new = pcl_to_ros(clusters_cloud)
   pub.publish(cloud_new)


 

if __name__ == '__main__':

    rospy.init_node("pipeline" , anonymous= True)
    rospy.Subscriber("/velodyne" , PointCloud2 , pcl_callback)
    pub = rospy.Publisher("/velodyne_pipeline" , PointCloud2 , queue_size=1)
    rospy.spin()