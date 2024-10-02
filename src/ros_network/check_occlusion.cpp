#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void postprocessed_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_occlusion");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud_proc = nh.subscribe("/cloud_postprocessed", 10, postprocessed_cloud_cb);
    ros::Publisher pub_occlusion = nh.advertise<sensor_msgs::PointCloud2>("/occlusion", 10);
    ros::Publisher pub_cable = nh.advertise<sensor_msgs::PointCloud2>("/cable", 10);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        if(cloud->size() != 0)
        {
            std::cout << std::endl << "CHECK OCCLUSION" << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            int indice_old, indice_new;
            Eigen::Vector3d differenza;

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

            sort_points(cloud, indice_old, indice_new, sortedCloud);
            std::cout << std::endl << "Sto verificando se ho un cavo oppure una occlusione..." << std::endl;
            std::cout << "Dimensione Nuvola Post Processata: " << cloud->points.size() << std::endl << "Dimensione nuvola ordinata: " << sortedCloud->points.size() << std::endl;
            double media = static_cast<double>(sortedCloud->points.size())/cloud->points.size();
            // if(cloud->points.size() - sortedCloud->points.size() >= 3)
            if(media < 0.70)
            {
                std::cout << "Altolà, questa è un'occlusione!!!" << std::endl;
                sensor_msgs::PointCloud2 occlusione;
                pcl::toROSMsg(*cloud, occlusione);
                occlusione.header.frame_id = "base_link";
                occlusione.header.stamp = ros::Time::now();
                pub_occlusion.publish(occlusione);
            }
            else
            {
                std::cout << "Questo è un cavo!!!" << std::endl;
                sensor_msgs::PointCloud2 cavo;
                pcl::toROSMsg(*cloud, cavo);
                cavo.header.frame_id = "base_link";
                cavo.header.stamp = ros::Time::now();
                pub_cable.publish(cavo);
            }
            cloud->points.clear();
            cloud->points.resize(0);
        }
        loop_rate.sleep();
    }

    return 0;
}