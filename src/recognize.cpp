#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

using namespace std::chrono_literals;

std::vector<std::string> findFilesContainingString(const std::string& directory, const std::string& searchString) {
    std::vector<std::string> matchingFiles;

    // Itera attraverso i file nella directory
    for(boost::filesystem::directory_iterator itr(directory); itr != boost::filesystem::directory_iterator(); ++itr) {
        if(boost::filesystem::is_regular_file(itr->path())) { // Controlla se è un file regolare
            std::string filename = itr->path().filename().string();
            if(filename.find(searchString) != std::string::npos) {
                matchingFiles.push_back(filename);
            }
        }
    }

    return matchingFiles;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "recognize");
    ros::NodeHandle nh;

    // Range del "bianco" per RGB: 130 < r,g,b < 175

    std::string directory = "/home/workstation2/ws_cross_modal/dataset/test_not_cable/";
    std::vector<std::string> paths = findFilesContainingString(directory, ".pcd");
    int num_file = paths.size();

    std::string path;
    int count_tavolo = 0;
    int count_cavo = 0;
    int count;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        count = 0;
        path = "/home/workstation2/ws_cross_modal/cluster35.pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }

        for(int i = 0; i < cloud->points.size(); i++)
        {
            if(cloud->points.at(i).r < 182 && cloud->points.at(i).r > 130)
                if(cloud->points.at(i).g < 182 && cloud->points.at(i).g > 130)
                    if(cloud->points.at(i).b < 182 && cloud->points.at(i).b > 130)
                        count++;
        }

        // std::cout << count << std::endl;

        double media = static_cast<double>(count)/cloud->points.size();
        std::cout << "Media: " << media << std::endl;
        if(media > 0.80)
        {
            std::cout << "TAVOLO" << std::endl;
            count_tavolo++;
        }
        else
        {
            std::cout << "NON TAVOLO" << std::endl;
            count_cavo++;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = cloud;
        pcl::visualization::CloudViewer viewer("Cluster viewer");
        viewer.showCloud(colored_cloud);

        while(!viewer.wasStopped ())
            std::this_thread::sleep_for(100us);


    // for(int j = 0; j < paths.size(); j++)
    // {
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     count = 0;
    //     path = directory + paths.at(j);
    //     if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }

    //     for(int i = 0; i < cloud->points.size(); i++)
    //     {
    //         if(cloud->points.at(i).r < 176 && cloud->points.at(i).r > 130)
    //             if(cloud->points.at(i).g < 176 && cloud->points.at(i).g > 130)
    //                 if(cloud->points.at(i).b < 176 && cloud->points.at(i).b > 130)
    //                     count++;
    //     }

    //     // std::cout << count << std::endl;

    //     double media = static_cast<double>(count)/cloud->points.size();
    //     std::cout << "Media: " << media << std::endl;
    //     if(media > 0.80)
    //     {
    //         std::cout << "TAVOLO" << std::endl;
    //         count_tavolo++;
    //     }
    //     else
    //     {
    //         std::cout << "CAVO" << std::endl;
    //         count_cavo++;
    //     }

    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = cloud;
    //     pcl::visualization::CloudViewer viewer("Cluster viewer");
    //     viewer.showCloud(colored_cloud);

    //     while(!viewer.wasStopped ())
    //         std::this_thread::sleep_for(100us);
            
    // }

    std::cout << "TAVOLO indovinati: " << count_tavolo << std::endl;
    std::cout << "NON TAVOLO indovinati: " << count_cavo << std::endl;

    return 0;
}