#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

void read_from_bag_and_send(ros::Publisher pub1, ros::Publisher pub2) {
   
    rosbag::Bag bag_grid, bag_point;
    std::vector<std::string> topics;
    topics.push_back(std::string("/pcl"));
    topics.push_back(std::string("/pcl2"));
    
    bag_grid.open("/home/workstation2/ws_cross_modal/PCL_grid.bag", rosbag::bagmode::Read);
    rosbag::View view(bag_grid, rosbag::TopicQuery(topics.at(0)));
    
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloudConstPtr pcl = m.instantiate<sensor_msgs::PointCloud>();
        if(pcl != NULL) {
            sensor_msgs::PointCloud PCL = *pcl;
            PCL.header.stamp = ros::Time::now();
            //std::cout << PCL << std::endl;
            pub1.publish(PCL);
        }
        
    }

    bag_grid.close();


    bag_point.open("/home/workstation2/ws_cross_modal/PCL_centroide.bag", rosbag::bagmode::Read);
    rosbag::View view2(bag_point, rosbag::TopicQuery(topics.at(1)));

    foreach(rosbag::MessageInstance const m, view2)
    {
        sensor_msgs::PointCloudConstPtr pcl2 = m.instantiate<sensor_msgs::PointCloud>();
        if(pcl2 != NULL) {
            sensor_msgs::PointCloud PCL2 = *pcl2;
            PCL2.header.stamp = ros::Time::now();
            //std::cout << PCL2 << std::endl;
            pub2.publish(PCL2);
        }
    }

    bag_point.close();
}

int main(int argc, char** argv) 
{

    ros::init(argc,argv,"visualize_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud>("/pcl", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud>("/pcl2", 1);
    ros::Rate loop_rate(20);
    int count = 0;
    while(ros::ok() && count < 20)
    {
        read_from_bag_and_send(pub1,pub2);
        //std::cout << "Point Cloud pubblicate" << std::endl;
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}