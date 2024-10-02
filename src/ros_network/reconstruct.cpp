#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void processed_cable_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "reconstruct");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    ros::Subscriber sub_interpolated = nh.subscribe("/cable", 10, processed_cable_cb);
    ros::Publisher pub_pcl_merged = nh.advertise<sensor_msgs::PointCloud2>("/pcl_merge", 10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        if(cloud->points.size() != 0)
        {
            std::cout << std::endl << "RECONSTRUCT" << std::endl;
            *partial_cloud += *cloud;
            sensor_msgs::PointCloud2 partial;
            pcl::toROSMsg(*cloud, partial);
            partial.header.frame_id = "base_link";
            partial.header.stamp = ros::Time::now();
            pub_pcl_merged.publish(partial);
            pcl::io::savePCDFile("/home/workstation2/pre_interpolazione" + boost::to_string(count) + ".pcd", *partial_cloud);
            cloud->clear();
            cloud->resize(0);
            count++;
        }
        loop_rate.sleep();
    }

    return 0;
}