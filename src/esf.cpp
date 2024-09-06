#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void esf_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

float computeVariance(pcl::PointCloud<pcl::ESFSignature640>::Ptr esf) {
    float mean = 1.0 / 640.0; // Poiché la somma è 1, la media è sempre 1/640
    float variance = 0.0;
    for (int i = 0; i < 640; ++i) {
        variance += pow(esf->points.at(0).histogram[i] - mean, 2);
    }
    return variance / 640.0;
}

float computeL2Norm(pcl::PointCloud<pcl::ESFSignature640>::Ptr esf) {
    float norm = 0.0;
    for (int i = 0; i < 640; ++i) {
        norm += esf->points.at(0).histogram[i] * esf->points.at(0).histogram[i];
    }
    return sqrt(norm);
}

float computeL1Distance(pcl::PointCloud<pcl::ESFSignature640>::Ptr esf1, pcl::PointCloud<pcl::ESFSignature640>::Ptr esf2)
{
    float sum = 0.0;
    for(int i = 0; i < 640; i++)
        sum += abs(esf1->points.at(0).histogram[i] - esf2->points.at(0).histogram[i]);

    return sum;
}

float computeChiSquareDistance(pcl::PointCloud<pcl::ESFSignature640>::Ptr esf1, pcl::PointCloud<pcl::ESFSignature640>::Ptr esf2)
{

    float sum = 0.0;
    for(int i = 0; i < 640; i++)
    {
        if((esf1->points.at(0).histogram[i] + esf2->points.at(0).histogram[i]) != 0)
            sum += (pow(esf1->points.at(0).histogram[i] - esf2->points.at(0).histogram[i], 2))/(esf1->points.at(0).histogram[i] + esf2->points.at(0).histogram[i]);
    }

    return sum;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "esf");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/cluster", 1, esf_cb);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/occlusione", 1);
    ros::Rate loop_rate(30);

    // while(ros::ok())
    // {
    std::vector<pcl::PointCloud<pcl::ESFSignature640>::Ptr> descriptor_vector;
    for(int i = 3; i < 9; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // cloud = visual_cloud;
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }
        pcl::ESFEstimation<pcl::PointXYZRGB, pcl::ESFSignature640> esf;
        pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
        esf.setInputCloud(cloud);
        esf.compute(*descriptor);
        descriptor_vector.push_back(descriptor);
        // for(int i = 0; i < 640; i++)
        // {
        //     std::cout << descriptor->points.at(0).histogram[i] << ",";
        // }

        // std::cout << "Descrittore ESF calcolato con " << descriptor->points.size() << " punti." << std::endl;
        // double sum = 0.0;
        // for(int i = 0; i < 640; i++)
        // {
        //     sum = sum + descriptor->points.at(0).histogram[i];
        // }

        // // sum = sum/640.0;
        // std::cout << sum << std::endl;
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

    // std::cout << "Distanza CHI tra Cavo 4 e Cavo 5: " << computeChiSquareDistance(descriptor_vector.at(0), descriptor_vector.at(1)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 4 e Cartoncino: " << computeChiSquareDistance(descriptor_vector.at(0), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra cavo 5 e Cartoncino: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra Cartoncino e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 4 e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(0), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 5 e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cartoncino e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(4)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 4 e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(0), descriptor_vector.at(4)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 5 e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(4)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo 5 e Cavo Verticale: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(5)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Verticale e Cartoncino: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(2)) << std::endl;

    
    

    // for(int i = 0; i < descriptor_vector.size(); i++)
    //     std::cout << "Varianza " << i+3 << ": " << computeVariance(descriptor_vector.at(i)) << std::endl;

    // std::cout << std::endl;

    // for(int i = 0; i < descriptor_vector.size(); i++)
    //     std::cout << "Norma L2 " << i+3 << ": " << computeL2Norm(descriptor_vector.at(i)) << std::endl;

    // std::cout << std::endl;

    // for(int i = 0; i < descriptor_vector.size(); i++)
    //     std::cout << "Mediana " << i+3 << ": " << computeMedian(descriptor_vector.at(i)) << std::endl;


    /* Aggiungere uso del descrittore per discriminare cavo o altro e publish */

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}
