#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

using namespace std::chrono_literals;

void segmentazione(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    pcl::io::loadPCDFile(path, *cloud_rgb);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_rgb);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	   clustering.setInputCloud(cloud_rgb);
	   clustering.setSearchMethod(kdtree);
    clustering.setMinClusterSize(600);
    clustering.setDistanceThreshold(10);
    clustering.setPointColorThreshold(6);
    clustering.setRegionColorThreshold(2);
    std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

    // For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud_rgb->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFile(fileName, *cluster);

		currentClusterNum++;
	}

    // Codice per concatenare i cluster
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/cluster2.pcd", *clu);
    // *concat += *clu;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/cluster3.pcd", *clu);
    // *concat += *clu;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/cluster5.pcd", *clu);
    // *concat += *clu;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/cluster6.pcd", *clu);
    // *concat += *clu;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/cluster7.pcd", *clu);
    // *concat += *clu; 
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/PCL_visuale_merge.pcd", *concat);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = clustering.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud (colored_cloud);

    while (!viewer.wasStopped ())
        std::this_thread::sleep_for(100us);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string path = "/home/workstation2/ws_cross_modal/PCL_visuale_spirale2_box.pcd";
    segmentazione(path,cloud);

    return 0;
}

// #include <iostream>

// #include <thread>

// #include <vector>


// #include <pcl/point_types.h>

// #include <pcl/io/pcd_io.h>

// #include <pcl/search/search.h>

// #include <pcl/search/kdtree.h>

// #include <pcl/visualization/cloud_viewer.h>

// #include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud

// #include <pcl/segmentation/region_growing_rgb.h>


// using namespace std::chrono_literals;


// int main ()
// {

//   pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//   pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//   if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/workstation2/ws_cross_modal/prova.pcd", *cloud) == -1 ) {std::cout << "Cloud reading failed." << std::endl; return (-1);}

//   pcl::IndicesPtr indices (new std::vector <int>);
//   pcl::removeNaNFromPointCloud (*cloud, *indices);
//   pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//   reg.setInputCloud (cloud);
//   reg.setIndices (indices);
//   reg.setSearchMethod (tree);
//   reg.setDistanceThreshold (10);
//   reg.setPointColorThreshold (6);
//   reg.setRegionColorThreshold (5);
//   reg.setMinClusterSize (600);
//   std::vector <pcl::PointIndices> clusters;
//   reg.extract (clusters);


//   pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

//   pcl::visualization::CloudViewer viewer ("Cluster viewer");

//   viewer.showCloud (colored_cloud);

//   while (!viewer.wasStopped ())

//   {

//     std::this_thread::sleep_for(100us);

//   }


//   return (0);

// }