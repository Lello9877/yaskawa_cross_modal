#include <ros/ros.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <pcl/surface/mls.h>
#define foreach BOOST_FOREACH

sensor_msgs::PointCloud2 cloud_visuale;

void bag_read_and_write() {
   
    rosbag::Bag bag;
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/depth/color/points"));
   
    bag.open("/home/workstation2/ws_cross_modal/bags/PCL_realsense.bag", rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics.at(0)));
   
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2ConstPtr pcl = m.instantiate<sensor_msgs::PointCloud2>();
        if(pcl != NULL) {
            cloud_visuale = *pcl;
            break;
        }
    }

    bag.close();

    bag.open("/home/workstation2/ws_cross_modal/bags/PCL_visuale.bag", rosbag::bagmode::Write);
    bag.write("/camera/depth/color/points", ros::Time::now(), cloud_visuale);
    bag.close();

}

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
    sensor_msgs::PointCloud2 PCL = bag_read<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_realsense.bag", "/camera/depth/color/points");
    bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_visuale.bag", "/camera/depth/color/points", PCL);
    sensor_msgs::PointCloud centr = bag_read<sensor_msgs::PointCloud>("/home/workstation2/ws_cross_modal/bags/PCL_centroide.bag", "/pcl2");
    sensor_msgs::PointCloud2 centr2;
    sensor_msgs::convertPointCloudToPointCloud2(centr, centr2);
    bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2.bag", "/pcl2", centr2);

    return 0;
}