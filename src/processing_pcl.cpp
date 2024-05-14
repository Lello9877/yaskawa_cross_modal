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
    // Load bun0.pcd -- should be available with the PCL archive in test 
    //pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);

    // // Set parameters
    mls.setInputCloud(cloud);
    //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.02);
    mls.setUpsamplingStepSize(0.01);
    //mls.setPointDensity(100);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    
    // Reconstruct
    mls.process(mls_points);

    // Save output
    //pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", mls_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", mls_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", *cloud_to_filter);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud_to_filter);
    filter.setLeafSize(0.008, 0.0, 0.0);
    filter.filter(*filteredCloud);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_filtered.pcd", *filteredCloud);

    return 0;
}