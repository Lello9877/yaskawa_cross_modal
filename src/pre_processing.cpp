#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pre_processing");
    ros::NodeHandle nh;
    std::string source_path = "/home/workstation2/ws_cross_modal/bags/5.pcd";
    std::string destination_path = "/home/workstation2/ws_cross_modal/bags/5_pre.pcd";
    std::string path = "/home/workstation2/ws_cross_modal/bags/";

    for(int i = 19; i <= 19; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string source_path = path + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(source_path, *cloud) != 0) { return -1; }
        std::vector<int> indice;
        float x_min = -0.25, x_max = 0.15;
        float y_min = -0.60, y_max = -0.10;
        float z_min = 0.08, z_max = 0.27;

        for(int i = 0; i < cloud->points.size(); i++)
        {
            // std::cout << cloud->points.at(i).z << std::endl;
            if(cloud->points.at(i).x < x_min || cloud->points.at(i).x > x_max || cloud->points.at(i).y < y_min || cloud->points.at(i).y > y_max || cloud->points.at(i).z < z_min || cloud->points.at(i).z > z_max)
            {
                // std::cout << "Sto aggiungendo il punto di indice " << i << std::endl;
                indice.push_back(i);
            }
        }

        std::sort(indice.begin(), indice.end());
        // std::cout << "Differenza: " << cloud->points.size() - indice.size() << std::endl;

        // for(int i = 0; i < indice.size(); i++)
        //     std::cout << indice.at(i) << std::endl;

        for(int i = 0; i < indice.size(); i++)
            cloud->points.erase(cloud->points.begin() + indice.at(i)-i);

        cloud->width = cloud->points.size();
        destination_path = path + boost::to_string(i) + "_pre.pcd";
        pcl::io::savePCDFile<pcl::PointXYZRGB>(destination_path, *cloud);
    }


    return 0;
}