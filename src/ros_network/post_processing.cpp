#include <ros/ros.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void downsampled_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "post_processing");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud_down = nh.subscribe("/cloud_downsampled", 10, downsampled_cloud_cb);
    ros::Publisher pub_cloud_post = nh.advertise<sensor_msgs::PointCloud2>("/cloud_postprocessed", 10);
    ros::Rate loop_rate(30);

    double distanza;
    // era tarato dmin = 0.02
    double dmin = 0.013;
    double soglia = dmin;
    std::vector<int> indice_vicino;
    int count = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        if(cloud->points.size() != 0)
        {
            std::cout << std::endl << "POST PROCESSING" << std::endl;
            for(int i = 0; i < cloud->points.size()-1; i++)
            {
                for(int j = i+1; j < cloud->points.size(); j++)
                {
                    distanza = euclideanDistance(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z, cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
                    if(distanza < soglia)
                    {
                        // indice_vicino.push_back(j);
                        //////////////////////////////
                        Eigen::Vector3d vicino_i(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
                        Eigen::Vector3d vicino_j(cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
                        cloud->points.erase(cloud->points.begin()+j);
                        Eigen::Vector3d medio = (vicino_i + vicino_j)/2;
                        cloud->points.at(i).x = medio.x();
                        cloud->points.at(i).y = medio.y();
                        cloud->points.at(i).z = medio.z();
                    }
                }
            }

            cloud->width = cloud->points.size();
            cloud->height = 1;
            pcl::io::savePCDFile("/home/workstation2/pcl_postprocessata" + boost::to_string(count) + ".pcd", *cloud);
            sensor_msgs::PointCloud2 cloud_post_processed;
            pcl::toROSMsg(*cloud, cloud_post_processed);
            cloud_post_processed.header.frame_id = "base_link";
            cloud_post_processed.header.stamp = ros::Time::now();
            pub_cloud_post.publish(cloud_post_processed);
            cloud->points.clear();
            cloud->points.resize(0);
            count++;
            std::this_thread::sleep_for(100ms);
        }
        loop_rate.sleep();
    }

    return 0;
}