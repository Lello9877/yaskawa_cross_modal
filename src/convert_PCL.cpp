#include <ros/ros.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#define foreach BOOST_FOREACH

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

template <typename T>
void bag_write(std::string path, std::string topic, T data) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write(topic,ros::Time::now(), data);
    bag.close();

}

int main(int argc, char** argv)
{

    ros::init(argc,argv,"convert_PCL");
    ros::NodeHandle nh;

    // NODO DA ELIMINARE: 38

    // sensor_msgs::PointCloud cloud_temp;
    // sensor_msgs::PointCloud2 cloud2;
    // cloud_temp = bag_read<sensor_msgs::PointCloud>("/home/workstation2/ws_cross_modal/bags/PCL_centr_spirale2_cen.bag", "/pcl2");
    // sensor_msgs::convertPointCloudToPointCloud2(cloud_temp, cloud2);
    // bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_cen.bag", "/pcl2", cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/prova.pcd", *cloud) != 0) { return -1; }
    cloud->points.erase(cloud->points.begin()+77);
    cloud->points.erase(cloud->points.begin()+71);
    cloud->width = cloud->width - 2;
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova2.pcd", *cloud);
    return 0;
}