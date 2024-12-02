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
#include <iostream>
#include <fstream>

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

    ros::init(argc, argv, "gasd");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::GASDSignature512> descriptor_retta;
    pcl::PointCloud<pcl::GASDSignature512> descriptor_spiral;

    // Lettura dei descrittori da file
    {
    descriptor_retta.resize(1);
    std::ifstream file("/home/workstation2/ws_cross_modal/descriptor_reference_retta.txt");
    if (!file.is_open()) {
        std::cerr << "Errore nell'aprire il file!" << std::endl;
        return -1;
    }
  
    
    std::string line;
    int index = 0;

    // Leggi ogni riga del file, converti il contenuto in float e assegnalo all'array
    while (std::getline(file, line) && index < 512) {
        try {
            descriptor_retta.at(0).histogram[index] = std::stof(line);  // Converti stringa in float
        } catch (const std::invalid_argument& e) {
            std::cerr << "Errore nella conversione da stringa a float alla riga " << index + 1 << std::endl;
            return -1;
        }
        ++index;
    }

    file.close();

    descriptor_spiral.resize(1);
    std::ifstream file2("/home/workstation2/ws_cross_modal/descriptor_reference_spiral.txt");
    if (!file2.is_open()) {
        std::cerr << "Errore nell'aprire il file!" << std::endl;
        return -1;
    }
  
    index = 0;

    // Leggi ogni riga del file, converti il contenuto in float e assegnalo all'array
    while (std::getline(file2, line) && index < 512) {
        try {
            descriptor_spiral.at(0).histogram[index] = std::stof(line);  // Converti stringa in float
        } catch (const std::invalid_argument& e) {
            std::cerr << "Errore nella conversione da stringa a float alla riga " << index + 1 << std::endl;
            return -1;
        }
        ++index;
    }

    file.close();
    }

    // for(int j = 0; j < 512; j++)
    //     std::cout << descriptor_retta.points.at(0).histogram[j] << ",";
    // std::cout << std::endl << std::endl;

    // for(int j = 0; j < 512; j++)
    //     std::cout << descriptor_spiral.points.at(0).histogram[j] << ",";
    // std::cout << std::endl << std::endl;

    std::vector<pcl::PointCloud<pcl::GASDSignature512>> descriptor_vector;
    for(int i = 1; i < 11; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string path = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(i) + ".pcd";
        // std::string path = "/home/workstation2/ws_cross_modal/cavo13.pcd";
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

    // pcl::PointCloud<pcl::GASDSignature512> descrittore_medio;
    // descrittore_medio.resize(1);

    // for(int i = 0; i < 512; i++)
    //     descrittore_medio.at(0).histogram[i] = 0.0;

    // for(int i = 0; i < descriptor_vector.size(); i++)
    //     for(int j = 0; j < 512; j++)
    //         descrittore_medio.at(0).histogram[j] += descriptor_vector.at(i).at(0).histogram[j];

    // // for(int i = 0; i < 512; i++)
    // //     std::cout << descrittore_medio.at(0).histogram[i] << ",";
    // // std::cout << std::endl << std::endl;

    // for(int i = 0; i < 512; i++)
    //     descrittore_medio.at(0).histogram[i] /= descriptor_vector.size();

    // for(int i = 0; i < 512; i++)
    //     std::cout << descrittore_medio.at(0).histogram[i] << ",";
    // std::cout << std::endl << std::endl;

    // // // Apri un file in modalità scrittura
    // // std::ofstream file("/home/workstation2/ws_cross_modal/histogram_cavo_retta.txt");
    // // if (!file.is_open()) {
    // //     std::cerr << "Errore nell'aprire il file!" << std::endl;
    // //     return -1;
    // // }

    // // // Salva l'istogramma nel file
    // // for (int i = 0; i < 512; ++i) {
    // //     file << descrittore_medio.at(0).histogram[i] << "\n";
    // // }

    // // file.close();

    // pcl::PointCloud<pcl::GASDSignature512> reference_descriptor_spiral;
    // reference_descriptor_spiral.resize(1);

    // std::ifstream file2("/home/workstation2/ws_cross_modal/histogram_cavo_spirale.txt");
    // if (!file2.is_open()) {
    //     std::cerr << "Errore nell'aprire il file!" << std::endl;
    //     return -1;
    // }
  
    // std::string line;
    // int index = 0;

    // // Leggi ogni riga del file, converti il contenuto in float e assegnalo all'array
    // while (std::getline(file2, line) && index < 512) {
    //     try {
    //         reference_descriptor_spiral.at(0).histogram[index] = std::stof(line);  // Converti stringa in float
    //     } catch (const std::invalid_argument& e) {
    //         std::cerr << "Errore nella conversione da stringa a float alla riga " << index + 1 << std::endl;
    //         return -1;
    //     }
    //     ++index;
    // }

    // for(int i = 0; i < 512; i++)
    //     std::cout << reference_descriptor_spiral.at(0).histogram[i] << ",";
    // std::cout << std::endl << std::endl;

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

    // std::cout << "Distanza CHI tra Cavo Orizzontale e Cavo Verticale: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(5)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Orizzontale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(2)) << std::endl;
    
    // std::cout << "Distanza CHI tra Cavo Orizzontale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Orizzontale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(1), descriptor_vector.at(4)) << std::endl << std::endl;

    // std::cout << "Distanza CHI tra Cavo Verticale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Verticale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Cavo Verticale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(5), descriptor_vector.at(4)) << std::endl;

    // std::cout << "Distanza CHI tra Occlusione e Tavolo: " <<  computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Occlusione e Tavolo Spezzato: " <<  computeChiSquareDistance(descriptor_vector.at(2), descriptor_vector.at(4)) << std::endl << std::endl;

    // std::cout << "Distanza CHI tra Spirale e Cavo Orizzontale: " <<  computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(1)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Cavo Verticale: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(5)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Occlusione: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(2)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Tavolo: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(3)) << std::endl;
    // std::cout << "Distanza CHI tra Spirale e Tavolo Spezzato: " << computeChiSquareDistance(descriptor_vector.at(6), descriptor_vector.at(4)) << std::endl;

    std::cout << "Distanza CHI tra Retta Riferimento e Cerchio: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(0)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Spirale: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(1)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Ovale: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(2)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Curva S: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(3)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Retta Verticale: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(4)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Retta Orizzontale: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(6)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Parabola: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(7)) << std::endl << std::endl;

    std::cout << "Distanza CHI tra Spirale Riferimento e Cerchio: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(0)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Spirale: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(1)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Ovale: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(2)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Curva S: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(3)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Retta Verticale: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(4)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Retta Orizzontale: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(6)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Parabola: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(7)) << std::endl << std::endl;

    std::cout << "Distanza CHI tra Retta Riferimento e Occlusione 1: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(5)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Occlusione 2: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(8)) << std::endl;
    std::cout << "Distanza CHI tra Retta Riferimento e Occlusione 3: " << computeChiSquareDistance(descriptor_retta, descriptor_vector.at(9)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Occlusione 1: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(5)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Occlusione 2: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(8)) << std::endl;
    std::cout << "Distanza CHI tra Spirale Riferimento e Occlusione 3: " << computeChiSquareDistance(descriptor_spiral, descriptor_vector.at(9)) << std::endl;

    return 0;

}