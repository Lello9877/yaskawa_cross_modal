#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "script");
    ros::NodeHandle nh;

    // ros::Rate loop_rate(30);

    // std::string path = "/home/workstation2/ws_cross_modal/img_spirale2/pcl_postprocessata4.pcd";
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::io::loadPCDFile("/home/workstation2/spirale2_offset.pcd", *visual);
    // Eigen::Vector3f temp;
    // // for(int i = 1; i < 3; i++)
    // // {
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/img_spirale2/pcl_postprocessata4.pcd", *cloud);
    //     for(int j = 0; j < cloud->points.size(); j++)
    //     {
    //         // std::cout << "Valori: " << cloud->points.at(j).x << cloud->points.at(j).y << cloud->points.at(j).z << std::endl;
    //         temp.x() = cloud->points.at(j).x;
    //         temp.y() = cloud->points.at(j).y;
    //         temp.z() = cloud->points.at(j).z;
    //         // temp.x() += 0.010; // 0.010
    //         // temp.y() -= 0.020; // 0.005
    //         temp.z() -= 0.010;
    //         cloud->points.at(j).x = temp.x();
    //         cloud->points.at(j).y = temp.y();
    //         cloud->points.at(j).z = temp.z();
    //     }
    //     cloud->width = cloud->points.size();
    //     cloud->height = 1;
    //     pcl::io::savePCDFile("/home/workstation2/spirale2_tactile_off2.pcd", *cloud);

    //     *visual += *cloud;
    //     // cloud->points.clear(); 
    //     // cloud->points.resize(0);
    // // // }
    // pcl::io::savePCDFile("/home/workstation2/spirale2_offset.pcd", *visual);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("/home/workstation2/spirale2_tactile_off2.pcd", *cloud1);
    pcl::io::loadPCDFile("/home/workstation2/spirale2_tactile_off.pcd", *cloud2);
    *cloud3 = *cloud1 + *cloud2;
    pcl::io::savePCDFile("/home/workstation2/spirale2_tactile_off.pcd", *cloud3);
    

    // pcl::io::savePCDFile("/home/workstation2/retta_offset.pcd", *visual);

            // std::cout << std::endl << "INTERPOLATOR" << std::endl;
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            // int indice_old, indice_new;
            // Eigen::Vector3d differenza;
            // int indice_iniziale = 0;
            // double distanza, soglia;
            // double prod;
            // double angolo;
            // bool trovato;
            // std::vector<int> indice_vicino;
            // std::vector<double> distanza_vicino;
            // std::vector<Eigen::Vector3d> direzione;
            // soglia = 0.03;
            // int count = 0;

            // pcl::io::loadPCDFile("/home/workstation2/spirale2_offset2.pcd", *cloud);
            // while(true && count < 10) // contatore di sicurezza per uscire dal ciclo 
            // {
            //     for(int i = 1; i < cloud->points.size(); i++)
            //     {
            //         distanza = euclideanDistance(cloud->points.at(indice_iniziale).x, cloud->points.at(indice_iniziale).y, cloud->points.at(indice_iniziale).z, cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
            //         if(distanza < soglia)
            //         {
            //             distanza_vicino.push_back(distanza);
            //             indice_vicino.push_back(i);
            //             // std::cout << "Ho trovato un vicino con indice: " << i << std::endl;
            //         }
            //     }

            //     for(int i = 0; i < indice_vicino.size(); i++)
            //     {
            //         differenza.x() = cloud->points.at(indice_vicino.at(i)).x - cloud->points.at(indice_iniziale).x;
            //         differenza.y() = cloud->points.at(indice_vicino.at(i)).y - cloud->points.at(indice_iniziale).y;
            //         differenza.z() = cloud->points.at(indice_vicino.at(i)).z - cloud->points.at(indice_iniziale).z;
            //         direzione.push_back(differenza);
            //     }
            //     std::cout << direzione.size() << std::endl;
            //     if(direzione.size() != 0)
            //         break;
            //     else { std::cout << "Nessuna direzione Trovata!! Incremento la distanza di soglia..." << std::endl; soglia += 0.01; count++; }
            // }

            // // std::cout << "Dimensione del vettore delle direzioni: " << direzione.size() << std::endl;

            // if(direzione.size() > 2)
            // {
            //     trovato = false;
            //     for(int i = 0; i < direzione.size()-1 && trovato == false; i++)
            //     {
            //         for(int j = i+1; j < direzione.size(); j++)
            //         {
            //             prod = direzione.at(i).x() * direzione.at(j).x() + direzione.at(i).y() * direzione.at(j).y() + direzione.at(i).z() * direzione.at(j).z();
            //             angolo = acos(prod/(direzione.at(i).norm()*direzione.at(j).norm()));
            //             if(angolo < 1  || angolo > 3.12414)
            //             {
            //                 trovato = true;
            //                 indice_old = indice_iniziale;
            //                 indice_new = indice_vicino.at(j);
            //                 break;
            //             }
            //         }
            //     }
            // } 
            // else
            // {
            //     indice_old = indice_iniziale; 
            //     indice_new = indice_vicino.at(0); 
            // }

            // std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;
            // sort_points(cloud, indice_old, indice_new, tempCloud);

            // Eigen::Vector3d point_old, point_new, point_temp;
            // point_old.x() = tempCloud->points.at(tempCloud->points.size()-1).x;
            // point_old.y() = tempCloud->points.at(tempCloud->points.size()-1).y;
            // point_old.z() = tempCloud->points.at(tempCloud->points.size()-1).z;
            // point_new.x() = tempCloud->points.at(tempCloud->points.size()-2).x;
            // point_new.y() = tempCloud->points.at(tempCloud->points.size()-2).y;
            // point_new.z() = tempCloud->points.at(tempCloud->points.size()-2).z;

            // for(int i = 0; i < cloud->points.size(); i++)
            // {   
            //     point_temp.x() = cloud->points.at(i).x;
            //     point_temp.y() = cloud->points.at(i).y;
            //     point_temp.z() = cloud->points.at(i).z;
            //     if(point_old == point_temp) { indice_old = i; }
            //     if(point_new == point_temp) { indice_new = i; }
            // }

            // std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;
            // sort_points(cloud, indice_old, indice_new, sortedCloud);
            // spline(sortedCloud, interpolatedCloud, 600);
            // pcl::io::savePCDFile("/home/workstation2/spirale2_interpolata6.pcd", *interpolatedCloud);
            // cloud->points.clear();
            // cloud->points.resize(0);
            // count++;

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::io::loadPCDFile("/home/workstation2/pcl_tattile.pcd", *cloud);

        // cloud->erase(cloud->begin() + cloud->points.size()-1);
        // pcl::io::savePCDFile("/home/workstation2/pcl_tattile_prova.pcd", *cloud);
    return 0;
}