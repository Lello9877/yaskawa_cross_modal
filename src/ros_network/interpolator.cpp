#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
void processed_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interpolator");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud_proc = nh.subscribe("/cloud_postprocessed", 1, processed_cloud_cb);

    while(ros::ok())
    {
        ros::spin();

        pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempInterpolatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        int indice_old, indice_new;
        Eigen::Vector3d differenza;
        std::string path_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola2_proc.pcd";
        std::string path_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_proc.pcd";

        // Porzione di codice per la scelta di una direzione
        int indice_iniziale = 0, count = 0;
        double distanza, soglia;
        double prod;
        double angolo;
        bool trovato;
        std::vector<int> indice_vicino;
        std::vector<double> distanza_vicino;
        std::vector<Eigen::Vector3d> direzione;
        soglia = 0.03;

        while(true && count < 10) // contatore di sicurezza per uscire dal ciclo 
        {
            for(int i = 1; i < cloud->points.size(); i++)
            {
                distanza = euclideanDistance(cloud->points.at(indice_iniziale).x, cloud->points.at(indice_iniziale).y, cloud->points.at(indice_iniziale).z, cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
                if(distanza < soglia)
                {
                    distanza_vicino.push_back(distanza);
                    indice_vicino.push_back(i);
                    // std::cout << "Ho trovato un vicino con indice: " << i << std::endl;
                }
            }

            for(int i = 0; i < indice_vicino.size(); i++)
            {
                differenza.x() = cloud->points.at(indice_vicino.at(i)).x - cloud->points.at(indice_iniziale).x;
                differenza.y() = cloud->points.at(indice_vicino.at(i)).y - cloud->points.at(indice_iniziale).y;
                differenza.z() = cloud->points.at(indice_vicino.at(i)).z - cloud->points.at(indice_iniziale).z;
                direzione.push_back(differenza);
            }
            std::cout << direzione.size() << std::endl;
            if(direzione.size() != 0)
                break;
            else { std::cout << "Nessuna direzione Trovata!! Incremento la distanza di soglia..." << std::endl; soglia += 0.01; count++; }
        }

        // std::cout << "Dimensione del vettore delle direzioni: " << direzione.size() << std::endl;

        if(direzione.size() > 2)
        {
            trovato = false;
            for(int i = 0; i < direzione.size()-1 && trovato == false; i++)
            {
                for(int j = i+1; j < direzione.size(); j++)
                {
                    prod = direzione.at(i).x() * direzione.at(j).x() + direzione.at(i).y() * direzione.at(j).y() + direzione.at(i).z() * direzione.at(j).z();
                    angolo = acos(prod/(direzione.at(i).norm()*direzione.at(j).norm()));
                    if(angolo < 1  || angolo > 3.12414)
                    {
                        trovato = true;
                        indice_old = indice_iniziale;
                        indice_new = indice_vicino.at(j);
                        break;
                    }
                }
            }
        } 
        else
        {
            indice_old = indice_iniziale; 
            indice_new = indice_vicino.at(0); 
        }

        std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;
        sort_points(cloud, indice_old, indice_new, tempCloud);
        spline(tempCloud, tempInterpolatedCloud, 400);
        // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_esp.pcd", *tempInterpolatedCloud);
        // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola2_esp.pcd", *tempInterpolatedCloud);

        Eigen::Vector3d point_old, point_new, point_temp;
        point_old.x() = tempCloud->points.at(tempCloud->points.size()-1).x;
        point_old.y() = tempCloud->points.at(tempCloud->points.size()-1).y;
        point_old.z() = tempCloud->points.at(tempCloud->points.size()-1).z;
        point_new.x() = tempCloud->points.at(tempCloud->points.size()-2).x;
        point_new.y() = tempCloud->points.at(tempCloud->points.size()-2).y;
        point_new.z() = tempCloud->points.at(tempCloud->points.size()-2).z;

        for(int i = 0; i < cloud->points.size(); i++)
        {   
            point_temp.x() = cloud->points.at(i).x;
            point_temp.y() = cloud->points.at(i).y;
            point_temp.z() = cloud->points.at(i).z;
            if(point_old == point_temp) { indice_old = i; }
            if(point_new == point_temp) { indice_new = i; }
        }
        std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;

        // indice_old = 26; indice_new = 27;
        sort_points(tempInterpolatedCloud, indice_old, indice_new, sortedCloud);
        spline(sortedCloud, interpolatedCloud, 400);
        // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola2_spline.pcd", *interpolatedCloud);
        // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_spline.pcd", *interpolatedCloud);

    }

    return 0;
}