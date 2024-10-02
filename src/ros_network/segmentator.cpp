#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void pre_processed_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentator");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/cloud_preprocessed", 10, pre_processed_cb);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/visual_cluster", 10);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        if(visual_cloud->points.size() != 0)
        {
            // std::cout << std::endl << "SEGMENTATOR" << std::endl;
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            kdtree->setInputCloud(visual_cloud);
            pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
            clustering.setInputCloud(visual_cloud);
            clustering.setSearchMethod(kdtree);
            clustering.setMinClusterSize(600);
            clustering.setDistanceThreshold(10);
            clustering.setPointColorThreshold(6);
            clustering.setRegionColorThreshold(5);
            std::vector <pcl::PointIndices> clusters;
            clustering.extract(clusters);

            int count_tavolo = 0;
            int currentClusterCavo = 0;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_vector;

            for(std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
            {
                std::cout << std::endl << "SEGMENTATOR" << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                    cluster->points.push_back(visual_cloud->points[*point]);
                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = true;

                int count = 0;

                for(int j = 0; j < cluster->points.size(); j++)
                {
                    if(cluster->points.at(j).r < 195 && cluster->points.at(j).r > 130)
                        if(cluster->points.at(j).g < 195 && cluster->points.at(j).g > 130)
                            if(cluster->points.at(j).b < 195 && cluster->points.at(j).b > 130)
                                count++;
                }

                // std::cout << count << std::endl;

                double media = static_cast<double>(count)/cluster->points.size();
                // std::cout << media << std::endl;
                if(media > 0.80)
                {
                    std::cout << "TAVOLO" << std::endl;
                    pcl::io::savePCDFile("/home/workstation2/tavolo" + boost::to_string(count_tavolo) + ".pcd", *cluster);
                    count_tavolo++;
                }
                else
                {    
                    std::cout << "NON TAVOLO" << std::endl;
                    currentClusterCavo++;
                    cluster_vector.push_back(cluster);
                }
            }

            std::cout << std::endl << "Ho individuato " << count_tavolo << " cluster TAVOLO" << std::endl;
            std::cout << "Ho individuato " << currentClusterCavo << " cluster NON TAVOLO" << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            if(currentClusterCavo < 1) 
                std::cout << "C'è un problema, nessuna point cloud riconosciuta come NON TAVOLO" << std::endl;

            else
            {
                for(int i = 0; i < cluster_vector.size(); i++)
                {

                    sensor_msgs::PointCloud2 cluster2;
                    cluster_temp = cluster_vector.at(i);
                    std::string path = "/home/workstation2/cluster" + boost::to_string(i) + ".pcd";
                    pcl::io::savePCDFile(path, *cluster_temp);
                    pcl::toROSMsg(*cluster_temp, cluster2);
                    cluster2.header.frame_id = "base_link";
                    cluster2.header.stamp = ros::Time::now();
                    std::cout << "Sto pubblicando il cluster " << i << std::endl;
                    // loop_rate.sleep();
                    pub_cluster.publish(cluster2);
                    std::this_thread::sleep_for(100ms);
                    // loop_rate.sleep();
                }
            }
            visual_cloud->points.clear();
            visual_cloud->points.resize(0);
            cluster_vector.clear();
            cluster_vector.resize(0);

        }
        loop_rate.sleep();
    }

    return 0;
}