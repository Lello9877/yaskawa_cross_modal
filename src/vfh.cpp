#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <pcl/features/cvfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

using namespace std::chrono_literals;

float computeL1Distance(pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh1, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh2)
{
    float sum = 0.0;
    for(int i = 0; i < 308; i++)
        sum += abs(vfh1->points.at(0).histogram[i] - vfh2->points.at(0).histogram[i]);

    return sum;
}

int main(int argc, char** argv)
{

    // Creare l'oggetto per l'estimazione del descrittore CVFH
    ros::init(argc, argv, "vfh");
    ros::NodeHandle nh;

    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> descriptor_vector;
    for(int i = 3; i < 9; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }

        // Calcolo delle Normali
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.03);
        ne.compute(*cloud_normals);
        pcl::PointCloud<pcl::Normal> clustering;
        pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/normal" + boost::to_string(i) + ".pcd", *cloud_normals);

        pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
        vfh.setInputCloud(cloud);
        vfh.setInputNormals(cloud_normals);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_cloud(new pcl::search::KdTree<pcl::PointXYZRGB>());
        vfh.setSearchMethod(tree_cloud);
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>());
        vfh.compute(*descriptor);
        descriptor_vector.push_back(descriptor);

        // std::cout << "Dimensione Descrittore: " << descriptor->size() << std::endl;

        // for(int j = 0; j < 308; j++)
        //     std::cout << descriptor->points.at(0).histogram[j] << ",";
        // std::cout << std::endl << std::endl;
    }

    std::cout << "Distanza L1 tra Cavo 4 e Cavo 5: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(1)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 4 e Cartoncino: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(2)) << std::endl;
    std::cout << "Distanza L1 tra cavo 5 e Cartoncino: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(2)) << std::endl;
    std::cout << "Distanza L1 tra Cartoncino e Tavolo: " << computeL1Distance(descriptor_vector.at(2), descriptor_vector.at(3)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 4 e Tavolo: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(3)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 5 e Tavolo: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(3)) << std::endl;
    std::cout << "Distanza L1 tra Cartoncino e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(2), descriptor_vector.at(4)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 4 e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(4)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 5 e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(4)) << std::endl;
    std::cout << "Distanza L1 tra Cavo 5 e Cavo Verticale: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(5)) << std::endl;
    std::cout << "Distanza L1 tra Cavo Verticale e Cartoncino: " << computeL1Distance(descriptor_vector.at(5), descriptor_vector.at(2)) << std::endl;

    return 0;

}