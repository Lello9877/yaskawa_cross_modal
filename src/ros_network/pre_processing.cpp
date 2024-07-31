#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void visual_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg) 
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pre_processing");
    ros::NodeHandle nh;
    ros::Subscriber sub_camera = nh.subscribe("/visual_cloud", 1, visual_cloud_cb);
    ros::Publisher pub_visual_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_preprocessed", 1);
    float x_min = -0.25, x_max = 0.15;
    float y_min = -0.60, y_max = -0.10;
    float z_min = 0.08, z_max = 0.27;
    std::vector<int> indice;
    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spin();
        // std::cout << "Oilloc!" << std::endl;
        for(int i = 0; i < visual_cloud->points.size(); i++)
        {
            if(visual_cloud->points.at(i).x < x_min || visual_cloud->points.at(i).x > x_max || visual_cloud->points.at(i).y < y_min || visual_cloud->points.at(i).y > y_max || visual_cloud->points.at(i).z < z_min || visual_cloud->points.at(i).z > z_max)
            {
                indice.push_back(i);
            }
        }

        std::sort(indice.begin(), indice.end());

        for(int i = 0; i < indice.size(); i++)
            visual_cloud->points.erase(visual_cloud->points.begin() + indice.at(i)-i);

        visual_cloud->width = visual_cloud->points.size();
        visual_cloud->height = 1;
        sensor_msgs::PointCloud2 cloud_preprocessed;
        pcl::toROSMsg(*visual_cloud, cloud_preprocessed);
        cloud_preprocessed.header.frame_id = "base_link";
        cloud_preprocessed.header.stamp = ros::Time::now();
        pub_visual_cloud.publish(cloud_preprocessed);
        visual_cloud->points.clear();
        visual_cloud->points.resize(0);
        loop_rate.sleep();
    }

    return 0;
}