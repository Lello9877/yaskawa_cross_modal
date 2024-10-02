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
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void postprocessed_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "euclidean_segmentation");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud_post = nh.subscribe("/tactile_cloud", 10, postprocessed_cloud_cb);
    ros::Publisher pub_euclidean_seg = nh.advertise<sensor_msgs::PointCloud2>("/tactile_cluster", 10);
    ros::Rate loop_rate(30);
    int count = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        if(cloud->size() != 0)
        {
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance(0.03); // 3cm
            ec.setMinClusterSize(2);
            ec.setMaxClusterSize(100);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            for(const auto& cluster : cluster_indices)
            {
                std::cout << std::endl << "EUCLIDEAN SEGMENTATION" << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(const auto& idx : cluster.indices) 
                {
                    cloud_cluster->push_back((*cloud)[idx]);
                }
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
                pcl::io::savePCDFile("/home/workstation2/euclidean_cluster" + boost::to_string(count) + ".pcd", *cloud_cluster);
                sensor_msgs::PointCloud2 cluster_cavo;
                pcl::toROSMsg(*cloud_cluster, cluster_cavo);
                cluster_cavo.header.frame_id = "base_link";
                cluster_cavo.header.stamp = ros::Time::now();
                pub_euclidean_seg.publish(cluster_cavo);
                std::this_thread::sleep_for(1000ms);
                count++;
                cloud_cluster->points.clear();
                cloud_cluster->points.resize(0);
                // loop_rate.sleep();
            }

            cloud->points.clear();
            cloud->points.resize(0);
        }
        loop_rate.sleep();
    }
    
    return 0;

}