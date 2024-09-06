#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void esf_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "shot");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/cluster", 1, esf_cb);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/occlusione", 1);
    ros::Rate loop_rate(30);


    std::vector<Eigen::VectorXf> esf_vector;
    for(int i = 4; i < 6; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // cloud = visual_cloud;
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }
        pcl::ESFEstimation<pcl::PointXYZRGB, pcl::ESFSignature640> esf;
        pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
        esf.setInputCloud(cloud);
        esf.compute(*descriptor);

        Eigen::VectorXf vec;
        vec.resize(640);
        for(int i = 0; i < 640; i++)
            vec(i) = descriptor->points.at(0).histogram[i];
        
        esf_vector.push_back(vec);
        // std::cout << "Descrittore ESF calcolato con " << descriptor->points.size() << " punti." << std::endl;
        // double sum = 0.0;
        // for(int i = 0; i < 640; i++)
        // {
        //     sum = sum + descriptor->points.at(0).histogram[i];
        // }

        // // sum = sum/640.0;
        // std::cout << sum << std::endl;
    }

    std::vector<Eigen::VectorXf> shot_vector;
    for(int i = 4; i < 6; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud2) != 0) { return -1; }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimation.setInputCloud(cloud2);
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setRadiusSearch(0.03); // Raggio di ricerca
        normal_estimation.compute(*cloud_normals);
        pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_estimation;
        pcl::PointCloud<pcl::SHOT352>::Ptr descriptor(new pcl::PointCloud<pcl::SHOT352>());
        shot_estimation.setInputCloud(cloud2);
        shot_estimation.setInputNormals(cloud_normals);
        shot_estimation.setSearchMethod(tree);
        shot_estimation.setRadiusSearch(0.02); // Raggio per calcolare il descrittore SHOT
        shot_estimation.compute(*descriptor);

        Eigen::VectorXf global_descriptor = Eigen::VectorXf::Zero(352);
        for (int j = 0; j < descriptor->size(); j++) {
            for (int k = 0; k < 352; k++) {
                global_descriptor[k] += descriptor->points[j].descriptor[k];
            }
        }
        global_descriptor /= descriptor->size();
        global_descriptor.resize(640);
        
        for(int i = 352; i < 640; i++)
            global_descriptor(i) = 0.0;

        shot_vector.push_back(global_descriptor);

    }

    int count = 0;
    for(int i = 4; i < 6; i++)
    {
        Eigen::MatrixXf mat(640, 2);
        mat.col(0) = esf_vector.at(count);
        mat.col(1) = shot_vector.at(count);
        count++;

        // std::cout << "Matrice originale:\n" << mat << std::endl;

        // Calcolare la media di ogni colonna
        Eigen::RowVectorXf col_mean = mat.colwise().mean();

        // Centrare la matrice sottraendo la media di ciascuna colonna
        Eigen::MatrixXf D_center = mat.rowwise() - col_mean;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(D_center, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf U = svd.matrixU();
        Eigen::VectorXf S = svd.singularValues();
        Eigen::MatrixXf V = svd.matrixV();  
        Eigen::VectorXf dr = U.col(0) * S(0);
        std::cout << "Valore singolare cluster " << i << ": "  << std::endl << S(0) << std::endl;
        
    }

    return 0;

}