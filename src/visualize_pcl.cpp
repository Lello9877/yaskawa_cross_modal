#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// void read_from_bag_and_send(ros::Publisher pub1, ros::Publisher pub2) {
   
//     rosbag::Bag bag_grid, bag_point;
//     std::vector<std::string> topics;
//     topics.push_back(std::string("/pcl"));
//     topics.push_back(std::string("/pcl2"));
    
//     bag_grid.open("/home/workstation2/ws_cross_modal/bags/PCL_grid.bag", rosbag::bagmode::Read);
//     rosbag::View view(bag_grid, rosbag::TopicQuery(topics.at(0)));
    
//     foreach(rosbag::MessageInstance const m, view)
//     {
//         sensor_msgs::PointCloudConstPtr pcl = m.instantiate<sensor_msgs::PointCloud>();
//         if(pcl != NULL) {
//             sensor_msgs::PointCloud PCL = *pcl;
//             PCL.header.stamp = ros::Time::now();
//             pub1.publish(PCL);
//         }
        
//     }

//     bag_grid.close();

//     bag_point.open("/home/workstation2/ws_cross_modal/bags/PCL_centroide.bag", rosbag::bagmode::Read);
//     rosbag::View view2(bag_point, rosbag::TopicQuery(topics.at(1)));

//     foreach(rosbag::MessageInstance const m, view2)
//     {
//         sensor_msgs::PointCloudConstPtr pcl2 = m.instantiate<sensor_msgs::PointCloud>();
//         if(pcl2 != NULL) {
//             sensor_msgs::PointCloud PCL2 = *pcl2;
//             PCL2.header.stamp = ros::Time::now();
//             pub2.publish(PCL2);
//         }
//     }

//     bag_point.close();
// }

sensor_msgs::PointCloud2 nuvola;

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
void bag_read_and_send(std::string path, std::string topic, ros::Publisher pub2) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    T val;

    foreach(rosbag::MessageInstance const m, view)
    {
        boost::shared_ptr<T> pcl = m.instantiate<T>();
        if(pcl != NULL) {
            val = *pcl;
            val.header.frame_id = "base_link";
            val.header.stamp = ros::Time::now();
            pub2.publish(val);
        }
    }

    bag.close();

}

int main(int argc, char** argv) 
{

    ros::init(argc,argv,"visualize_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("/centroide", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/visuale", 1);
    ros::Rate loop_rate(20);
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 centr;
    int count = 0;
    while(ros::ok() && count < 20)
    {
        count++;
        centr = bag_read<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.bag", "/pcl2");
        centr.header.frame_id = "base_link";
        centr.header.stamp = ros::Time::now();
        pub1.publish(centr);
        bag_read_and_send<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_realsense_spirale.bag", "/cloud2_base", pub2);
        loop_rate.sleep();
    }
    return 0;

}