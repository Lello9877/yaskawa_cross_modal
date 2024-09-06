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

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void pre_processed_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

double cluster_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu2)
{
    double distance = 500.0;
    double temp_distance = 0.0;
    for(int i = 0; i < clu1->points.size(); i++)
    {
        for(int j = 0; j < clu2->points.size(); j++)
        {
            temp_distance = euclideanDistance(clu1->points.at(i).x, clu1->points.at(i).y, clu1->points.at(i).z, clu2->points.at(j).x, clu2->points.at(j).y, clu2->points.at(j).z);
            if(temp_distance < distance)
                distance = temp_distance;
            if(temp_distance < 0.008) 
                return distance;
        }
    }
    return distance;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/cloud_preprocessed", 1, pre_processed_cb);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
    ros::Publisher pub_cluster_cavo = nh.advertise<sensor_msgs::PointCloud2>("cluster_cavo", 1);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spin();
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
            // ...add all its points to a new cloud...
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                cluster->points.push_back(visual_cloud->points[*point]);
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;

            int count = 0;

            for(int j = 0; j < cluster->points.size(); j++)
            {
                if(cluster->points.at(j).r < 176 && cluster->points.at(j).r > 130)
                    if(cluster->points.at(j).g < 176 && cluster->points.at(j).g > 130)
                        if(cluster->points.at(j).b < 176 && cluster->points.at(j).b > 130)
                            count++;
            }

            // std::cout << count << std::endl;

            double media = static_cast<double>(count)/cluster->points.size();
            std::cout << media << std::endl;
            if(media > 0.80)
            {
                std::cout << "TAVOLO" << std::endl;
                count_tavolo++;
            }
            else
            {    
                std::cout << "NON TAVOLO" << std::endl;
                currentClusterCavo++;
                cluster_vector.push_back(cluster);

                // ...and save it to disk.
                // if (cluster->points.size() <= 0)
                //     break;
                // std::cout << "Cluster " << currentClusterCavo << " has " << cluster->points.size() << " points." << std::endl;
                // std::string fileName = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(currentClusterCavo) + ".pcd";
                // pcl::io::savePCDFile(fileName, *cluster);

                // sensor_msgs::PointCloud2 cluster2;
                // pcl::toROSMsg(*cluster, cluster2);
                // cluster2.header.frame_id = "base_link";
                // cluster2.header.stamp = ros::Time::now();
                // pub_cluster.publish(cluster2);
            }
        }

        std::cout << "Ho individuato " << count_tavolo << " cluster TAVOLO" << std::endl;
        std::cout << "Ho individuato " << currentClusterCavo << " cluster NON TAVOLO" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(currentClusterCavo < 1) std::cout << "C'è un problema, nessuna point cloud riconosciuta come NON TAVOLO" << std::endl;
        else if(currentClusterCavo == 1)
        {
            sensor_msgs::PointCloud2 cluster2;
            cluster_temp = cluster_vector.at(0);
            pcl::toROSMsg(*cluster_temp, cluster2);
            cluster2.header.frame_id = "base_link";
            cluster2.header.stamp = ros::Time::now();
            pub_cluster_cavo.publish(cluster2);
        }
        else
        {
            for(int i = 0; i < cluster_vector.size(); i++)
            {
                sensor_msgs::PointCloud2 cluster2;
                cluster_temp = cluster_vector.at(i);
                pcl::toROSMsg(*cluster_temp, cluster2);
                cluster2.header.frame_id = "base_link";
                cluster2.header.stamp = ros::Time::now();
                pub_cluster.publish(cluster2);
            }
        }
        visual_cloud->points.clear();
        visual_cloud->points.resize(0);
        cluster_vector.clear();
        cluster_vector.resize(0);
        loop_rate.sleep();
    }

    return 0;
}