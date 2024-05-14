#include <ros/ros.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types_conversion.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/icp.h>
#define foreach BOOST_FOREACH

template <typename T>
void bag_write(std::string path, std::string topic, T data) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write(topic,ros::Time::now(), data);
    bag.close();

}

template <typename T>
T bag_read(std::string path, std::string topic) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    T val;

    foreach(rosbag::MessageInstance const m, view)
    {
        boost::shared_ptr<T> pcl = m.instantiate<T>();
        if(pcl != NULL) {
            val = *pcl;
        }
    }

    bag.close();
    return val;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "processing_pcl");
    ros::NodeHandle nh;

    // sensor_msgs::PointCloud2 PCL = bag_read<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_realsense_cerchio.bag", "/camera/depth/color/points");
    // bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio.bag", "/camera/depth/color/points", PCL);
    // sensor_msgs::PointCloud centr = bag_read<sensor_msgs::PointCloud>("/home/workstation2/ws_cross_modal/bags/PCL_centroide_retta.bag", "/pcl2");
    // sensor_msgs::PointCloud2 centr2;
    // sensor_msgs::convertPointCloudToPointCloud2(centr, centr2);
    // bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.bag", "/pcl2", centr2);

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud);
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)

    // Upsampling tattile retta
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.02);
    mls.setUpsamplingStepSize(0.01);
    mls.setPointDensity(100000);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(mls_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", mls_points);

    // Filtraggio VoxelGrid tattile retta
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", *cloud_to_filter);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud_to_filter);
    filter.setLeafSize(0.0005, 0.0, 0.0);
    filter.filter(*filteredCloud);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_filtered.pcd", *filteredCloud);

    // Segmentazione visuale retta
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta.pcd", *cloud_rgb);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_rgb);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud_rgb);
	clustering.setSearchMethod(kdtree);
    clustering.setMinClusterSize(100);
    clustering.setDistanceThreshold(10);
    clustering.setPointColorThreshold(6);
    clustering.setRegionColorThreshold(5);
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
		std::string fileName = "/home/workstation2/ws_cross_modal/bags/cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFile(fileName, *cluster);

		currentClusterNum++;
	}

    // Filtraggio visuale retta
    int num_points = 684;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filt(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/cluster9.pcd", *cloud_to_filt);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::RandomSample<pcl::PointXYZRGB> sampler;
    sampler.setInputCloud(cloud_to_filt);
    sampler.setSample(num_points);
    sampler.filter(*cloud_filtered);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_filtered.pcd", *cloud_filtered);

    // ICP retta
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_filtered.pcd", *sourceCloud);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_filtered.pcd", *targetCloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(sourceCloud);
	registration.setInputTarget(targetCloud);
	registration.align(*finalCloud);

	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
        pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_icp_retta.pcd", *finalCloud);
	}
	else std::cout << "ICP did not converge." << std::endl;

    return 0;
}