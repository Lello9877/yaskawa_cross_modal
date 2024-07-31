#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
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
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/cloud_preprocessed", 1, pre_processed_cb);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
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

        // For every cluster...
        int currentClusterNum = 1;
        for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
        {
            // ...add all its points to a new cloud...
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                cluster->points.push_back(visual_cloud->points[*point]);
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;

            int count = 0;

            for(int i = 0; i < cluster->points.size(); i++)
            {
                if(cluster->points.at(i).r < 175 && cluster->points.at(i).r > 130)
                    if(cluster->points.at(i).g < 176 && cluster->points.at(i).g > 130)
                        if(cluster->points.at(i).b < 176 && cluster->points.at(i).b > 130)
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
                std::cout << "CAVO" << std::endl;

            // ...and save it to disk.
            // if (cluster->points.size() <= 0)
            //     break;
            // std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
            // std::string fileName = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(currentClusterNum) + ".pcd";
            // pcl::io::savePCDFile(fileName, *cluster);
            currentClusterNum++;
            sensor_msgs::PointCloud2 cluster2;
            pcl::toROSMsg(*cluster, cluster2);
            cluster2.header.frame_id = "base_link";
            cluster2.header.stamp = ros::Time::now();
            pub_cluster.publish(cluster2);
            visual_cloud->points.clear();
            visual_cloud->points.resize(0);
            loop_rate.sleep();
        }
    }

    return 0;
}