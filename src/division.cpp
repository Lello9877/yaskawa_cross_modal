#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

int 
main ()
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("/home/workstation2/pcl_postprocessata3.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

//   // Create the filtering object: downsample the dataset using a leaf size of 1cm
//   pcl::VoxelGrid<pcl::PointXYZ> vg;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   vg.setInputCloud (cloud);
//   vg.setLeafSize (0.01f, 0.01f, 0.01f);
//   vg.filter (*cloud_filtered);
//   std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

//   // Create the segmentation object for the planar model and set all the parameters
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//   pcl::PCDWriter writer;
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   seg.setDistanceThreshold (0.02);

//   int nr_points = (int) cloud_filtered->size ();
//   while (cloud_filtered->size () > 0.3 * nr_points)
//   {
//     // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud (cloud_filtered);
//     seg.segment (*inliers, *coefficients);
//     if (inliers->indices.size () == 0)
//     {
//       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//       break;
//     }

//     // Extract the planar inliers from the input cloud
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud (cloud_filtered);
//     extract.setIndices (inliers);
//     extract.setNegative (false);

//     // Get the points associated with the planar surface
//     extract.filter (*cloud_plane);
//     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

//     // Remove the planar inliers, extract the rest
//     extract.setNegative (true);
//     extract.filter (*cloud_f);
//     *cloud_filtered = *cloud_f;
//   }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (2);
  ec.setMaxClusterSize (100);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) 
    {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << j;
    pcl::io::savePCDFile("/home/workstation2/cloud_cluster" + boost::to_string(j) + ".pcd", *cloud_cluster);
    j++;
  }

  // Visualizzatore PCL
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster Viewer"));
  viewer->setBackgroundColor(0, 0, 0);  // Colore di sfondo nero

  // PointCloud con i cluster colorati
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  int cluster_id = 0;
  std::vector<uint8_t> colors = {255, 0, 0};  // Colore iniziale (rosso)

  for (const auto& indices : cluster_indices) {
    // Creazione di un colore random per ogni cluster
    uint8_t r = rand() % 256;
    uint8_t g = rand() % 256;
    uint8_t b = rand() % 256;

    for (const auto& idx : indices.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud->points[idx].x;
      point.y = cloud->points[idx].y;
      point.z = cloud->points[idx].z;
      point.r = r;
      point.g = g;
      point.b = b;
      colored_cloud->points.push_back(point);
    }
    cluster_id++;
  }

  colored_cloud->width = colored_cloud->points.size();
  colored_cloud->height = 1;
  colored_cloud->is_dense = true;

  // Aggiunta della point cloud colorata al viewer
  viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored cloud");

  // Impostazioni del visualizzatore
//   viewer->addCoordinateSystem(1.0);
//   viewer->initCameraParameters();
  
  // Ciclo di visualizzazione
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return (0);
}