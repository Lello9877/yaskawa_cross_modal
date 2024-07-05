#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

double calcDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double x = x1 - x2; // calculating number to square in next step
    double y = y1 - y2;
    double z = z1 - z2;
    double dist;

    dist = pow(x,2) + pow(y,2) + pow(z,2); // calculating Euclidean distance
    dist = sqrt(dist);

    return dist;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "visual_post_processing");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_grid.pcd", *cloud) != 0) { return -1; }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_grid.pcd", *cloud) != 0) { return -1; }
    double distanza;
    double dmin = 0.020;
    double soglia = dmin;

    for(int i = 0; i < cloud->points.size()-1; i++)
    {
        for(int j = i+1; j < cloud->points.size(); j++)
        {
            distanza = calcDistance(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z, cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
            if(distanza < soglia)
            {
                Eigen::Vector3d vicino_i(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
                Eigen::Vector3d vicino_j(cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
                cloud->points.erase(cloud->points.begin()+j);
                Eigen::Vector3d medio = (vicino_i + vicino_j)/2;
                cloud->points.at(i).x = medio.x();
                cloud->points.at(i).y = medio.y();
                cloud->points.at(i).z = medio.z();
            }
        }
    }

    cloud->width = cloud->points.size();
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_proc.pcd", *cloud);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_proc.pcd", *cloud);

    // Porzione di codice per visualizzare in modo agevole i punti con delle linee verticali
    // pcl::PointCloud<pcl::PointXYZ>::Ptr grid(new pcl::PointCloud<pcl::PointXYZ>);
    // double x_min, y_min, x_max, y_max;
    // x_min = cloud->points.at(0).x;
    // x_max = cloud->points.at(0).x;
    // y_max = cloud->points.at(0).y;
    // y_min = cloud->points.at(0).y;

    // for(int i = 1; i < cloud->points.size(); i++)
    // {
    //     if(cloud->points.at(i).x <= x_min)
    //         x_min = cloud->points.at(i).x;
    //     if(cloud->points.at(i).x >= x_max)
    //         x_max = cloud->points.at(i).x;            
    //     if(cloud->points.at(i).y <= y_min)
    //         y_min = cloud->points.at(i).y;
    //     if(cloud->points.at(i).y >= y_max)
    //         y_max = cloud->points.at(i).y;
    // }

    // pcl::PointXYZ punto;
    // int num_points = 25;
    // int divx, divy;
    // double div;
    // double delta;

    // div = (x_max-x_min)/dmin;
    // if(div < 1) divx = 1;
    // else divx = ceil(div); /*divx = floor(div)*/

    // div = (y_max-y_min)/dmin;
    // if(div < 1) divy = 1;
    // else divy = ceil(div);

    // *grid = *cloud;
    // punto.z = 0;
    // delta = (y_max - y_min)/num_points;

    // for(int i = 0; i < divx; i++) 
    // {
    //     punto.x = x_min + i*dmin;
    //     punto.y = y_min;
    //     for(int j = 0; j < num_points; j++)
    //     {
    //         grid->points.push_back(punto);
    //         punto.y += delta;
    //     }        
    // }

    // delta = (x_max - x_min)/num_points;
    // for(int i = 0; i < divy; i++) 
    // {
    //     punto.y = y_min + i*dmin;
    //     punto.x = x_min;
    //     for(int j = 0; j < num_points; j++)
    //     {
    //         grid->points.push_back(punto);
    //         punto.x += delta;
    //     }        
    // }

    // grid->width = grid->points.size();
    // for(int i = 0; i < grid->points.size(); i++)
    //     grid->points.at(i).z = 0;
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_grigliata.pcd", *grid);

    // int num = 14;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr prova(new pcl::PointCloud<pcl::PointXYZ>());
    // prova->points.resize(num);
    // prova->width = num;
    // prova->height = 1;
    // for(int i = 0; i < num; i++)
    //     prova->points.at(i) = cloud->points.at(i);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova.pcd", *prova);


    return 0;
}