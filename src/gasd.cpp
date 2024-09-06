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
#include <pcl/features/gasd.h>

float computeL1Distance(pcl::PointCloud<pcl::GASDSignature512> gasd1, pcl::PointCloud<pcl::GASDSignature512> gasd2)
{
    float sum = 0.0;
    for(int i = 0; i < 512; i++)
        sum += abs(gasd1.points.at(0).histogram[i] - gasd2.points.at(0).histogram[i]);

    return sum;
}

float computeChiSquareDistance(pcl::PointCloud<pcl::GASDSignature512> gasd1, pcl::PointCloud<pcl::GASDSignature512> gasd2)
{

    float sum = 0.0;
    for(int i = 0; i < 512; i++)
    {
        if((gasd1.points.at(0).histogram[i] + gasd2.points.at(0).histogram[i]) != 0)
            sum += (pow(gasd1.points.at(0).histogram[i] - gasd2.points.at(0).histogram[i], 2))/(gasd1.points.at(0).histogram[i] + gasd2.points.at(0).histogram[i]);
    }

    return sum;
}

int main(int argc, char** argv)
{

    // Creare l'oggetto per l'estimazione del descrittore CVFH
    ros::init(argc, argv, "gasd");
    ros::NodeHandle nh;

    std::vector<pcl::PointCloud<pcl::GASDSignature512>> descriptor_vector;
    for(int i = 3; i < 10; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }

        pcl::GASDEstimation<pcl::PointXYZRGB, pcl::GASDSignature512> gasd;
        gasd.setInputCloud(cloud);
        pcl::PointCloud<pcl::GASDSignature512> descriptor;
        gasd.compute(descriptor);
        descriptor_vector.push_back(descriptor);
        Eigen::Matrix4f trans = gasd.getTransform();
        // std::cout << "Dimensione Descrittore: " << descriptor.size() << std::endl;

        // for(int j = 0; j < 512; j++)
        //     std::cout << descriptor.points.at(0).histogram[j] << ",";
        // std::cout << std::endl << std::endl;
    }

    // std::cout << "Distanza L1 tra 2 Cavi Orizzontali: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(1)) << std::endl;
    // // std::cout << "Distanza L1 tra Cavo 4 e Cartoncino: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza L1 tra Cavo Orizzontale e Cartoncino: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza L1 tra Cartoncino e Tavolo: " << computeL1Distance(descriptor_vector.at(2), descriptor_vector.at(3)) << std::endl;
    // // std::cout << "Distanza L1 tra Cavo 4 e Tavolo: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza L1 tra Cavo Orizzontale e Tavolo: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza L1 tra Cartoncino e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(2), descriptor_vector.at(4)) << std::endl;
    // // std::cout << "Distanza L1 tra Cavo 4 e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(0), descriptor_vector.at(4)) << std::endl;
    // std::cout << "Distanza L1 tra Cavo Orizzontale e Tavolo Spezzato: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(4)) << std::endl;
    // std::cout << "Distanza L1 tra Cavo Orizzontale e Cavo Verticale: " << computeL1Distance(descriptor_vector.at(1), descriptor_vector.at(5)) << std::endl;
    // std::cout << "Distanza L1 tra Cavo Verticale e Cartoncino: " << computeL1Distance(descriptor_vector.at(5), descriptor_vector.at(2)) << std::endl;

    std::cout << "Distanza CHI tra Cavo Orizzontale e Cavo Verticale: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(5)) << std::endl;
    std::cout << "Distanza CHI tra Cavo Orizzontale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(2)) << std::endl;
    
    // std::cout << "Distanza CHI tra Cavo Orizzontale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Orizzontale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(4)) << std::endl << std::endl;

    std::cout << "Distanza CHI tra Cavo Verticale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Verticale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Verticale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(4)) << std::endl;

    // std::cout << "Distanza CHI tra Occlusione e Tavolo: " <<  computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Occlusione e Tavolo Spezzato: " <<  computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(4)) << std::endl << std::endl;

    std::cout << "Distanza CHI tra Spirale e Cavo Orizzontale: " <<  computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(1)) << std::endl;
    std::cout << "Distanza CHI tra Spirale e Cavo Verticale: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(5)) << std::endl;
    std::cout << "Distanza CHI tra Spirale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(4)) << std::endl;

    return 0;

}