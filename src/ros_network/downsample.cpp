#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "yaskawa_cross_modal/utility.h"

bool arrivato = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void visual_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
    std::cout << "arrivato beefore " << arrivato << std::endl;
    std::cout << "CALLBACK DOWNSAMPLE" << std::endl;
    arrivato = true;

}

void tactile_cloud_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "downsample");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/visual_cluster", 10, visual_cloud_cb);
    ros::Subscriber sub_tactile_cloud = nh.subscribe("/tactile_cluster", 10, tactile_cloud_cb);
    ros::Publisher pub_cloud_down = nh.advertise<sensor_msgs::PointCloud2>("/cloud_downsampled", 10);
    ros::Rate loop_rate(30);

    // Downsampling delle point cloud visuali
    double dmin = 0.015;
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        // std::cout << "Sono nel while, la dimensione è: " << cloud->points.size() << std::endl;
        // std::cout << "arrivato while" << arrivato << std::endl;

        if(cloud->points.size() != 0)
        {
            std::cout << std::endl << "DOWNSAMPLE" << std::endl;
            // Ricerca x e y per definire la griglia
            double x_min, y_min, x_max, y_max;
            x_min = cloud->points.at(0).x;
            x_max = cloud->points.at(0).x;
            y_max = cloud->points.at(0).y;
            y_min = cloud->points.at(0).y;

            for(int i = 1; i < cloud->points.size(); i++)
            {
                if(cloud->points.at(i).x <= x_min)
                    x_min = cloud->points.at(i).x;
                if(cloud->points.at(i).x >= x_max)
                    x_max = cloud->points.at(i).x;            
                if(cloud->points.at(i).y <= y_min)
                    y_min = cloud->points.at(i).y;
                if(cloud->points.at(i).y >= y_max)
                    y_max = cloud->points.at(i).y;
            }

            std::cout << "Coordinata x minima: " << x_min << std::endl;
            std::cout << "Coordinata x massima: " << x_max << std::endl;
            std::cout << "Coordinata y minima: " << y_min << std::endl;
            std::cout << "Coordinata y massima: " << y_max << std::endl; 

            std::vector<int> indice_voxel;
            std::vector<int> indice_selezionato;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);

            int divx, divy;
            double div, sumx, sumy, sumz;   

            div = (x_max-x_min)/dmin;
            if(div < 1) divx = 1;
            else divx = ceil(div); /*divx = floor(div)*/

            div = (y_max-y_min)/dmin;
            if(div < 1) divy = 1;
            else divy = ceil(div);
            
            std::cout << "divx: " << divx << std::endl;
            std::cout << "divy: " << divy << std::endl;

            pcl::PointXYZRGB centroide;
            for(int i = 0; i < divx; i++)
            {
                for(int j = 0; j < divy; j++)
                {
                    for(int k = 0; k < cloud->points.size(); k++) 
                    {
                        if(cloud->points.at(k).x < x_min + (i+1)*dmin && cloud->points.at(k).x > x_min + i*dmin)
                            if(cloud->points.at(k).y < y_min + (j+1)*dmin && cloud->points.at(k).y > y_min + j*dmin)
                                indice_selezionato.push_back(k);
                    }
                    if(indice_selezionato.size() != 0)
                    {
                        sumx = 0; sumy = 0; sumz = 0;
                        for(int m = 0; m < indice_selezionato.size(); m++)
                        {
                            sumx += cloud->points.at(indice_selezionato.at(m)).x;
                            sumy += cloud->points.at(indice_selezionato.at(m)).y;
                            sumz += cloud->points.at(indice_selezionato.at(m)).z;
                        }
                        sumx /= indice_selezionato.size();
                        sumy /= indice_selezionato.size();
                        sumz /= indice_selezionato.size();
                        centroide.x = sumx;
                        centroide.y = sumy;
                        centroide.z = sumz;
                        cloud_voxel->points.push_back(centroide);
                    }
                    indice_selezionato.clear();
                    indice_selezionato.resize(0);
                }
            }

            cloud_voxel->width = cloud_voxel->points.size();
            cloud_voxel->height = 1;
            pcl::io::savePCDFile("/home/workstation2/pcl_downsample" + boost::to_string(count) + ".pcd", *cloud_voxel);
            sensor_msgs::PointCloud2 cloud_down;
            pcl::toROSMsg(*cloud_voxel, cloud_down);
            cloud_down.header.frame_id = "base_link";
            cloud_down.header.stamp = ros::Time::now();
            pub_cloud_down.publish(cloud_down);
            cloud->points.clear();
            cloud->points.resize(0);
            count++;
            arrivato = false;
        }
        loop_rate.sleep();
    }

    return 0;
}