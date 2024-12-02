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

    ros::init(argc, argv, "post_process");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proc(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/pcl_downsample3.pcd", *cloud) != 0) { return -1; }
    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_grid.pcd", *cloud) != 0) { return -1; }
    double distanza;
    // dmin = 0.02;
    double dmin = 0.013;
    double soglia = dmin;
    std::vector<int> indice_vicino;
    cloud_proc->points.resize(cloud->points.size());

    int count = 0;
    for(int i = 0; i < cloud->points.size()-1; i++)
    {
        for(int j = i+1; j < cloud->points.size(); j++)
        {
            distanza = calcDistance(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z, cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
            if(distanza < soglia)
            {
                // indice_vicino.push_back(j);
                //////////////////////////////
                Eigen::Vector3d vicino_i(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
                Eigen::Vector3d vicino_j(cloud->points.at(j).x, cloud->points.at(j).y, cloud->points.at(j).z);
                cloud->points.erase(cloud->points.begin()+j);
                Eigen::Vector3d medio = (vicino_i + vicino_j)/2;
                cloud->points.at(i).x = medio.x();
                cloud->points.at(i).y = medio.y();
                cloud->points.at(i).z = medio.z();
            }
        }

        // if(indice_vicino.size() >= 1)
        // {
        //     Eigen::Vector3d sum_point(0.0,0.0,0.0), temp(0.0,0.0,0.0);

        //     for(int k = 0; k < indice_vicino.size(); k++) 
        //     {
        //         temp.x() = cloud->points.at(indice_vicino.at(k)).x;
        //         temp.y() = cloud->points.at(indice_vicino.at(k)).y;
        //         temp.z() = cloud->points.at(indice_vicino.at(k)).z;
        //         sum_point += temp;
        //     }
        //     sum_point /= indice_vicino.size();

        //     std::sort(indice_vicino.begin(), indice_vicino.end());
            // std::cout << indice_vicino.size() << std::endl;

            // int ultimo_indice = -1;
            // for(int l = 0; l < indice_vicino.size(); l++)
            // {
            //     // if(indice_vicino.at(l) > ultimo_indice)
            //         cloud->points.erase(cloud->points.begin() + indice_vicino.at(l) - l);
            //     // else
            //     //     cloud->points.erase(cloud->points.begin() + indice_vicino.at(l));
            //     // ultimo_indice = indice_vicino.at(l);
            // }

            // cloud_proc->points.at(i).x = sum_point.x();
            // cloud_proc->points.at(i).y = sum_point.y();
            // cloud_proc->points.at(i).z = sum_point.z();
        // }

        // cloud_proc->points.at(i) = cloud->points.at(i);
        // indice_vicino.clear();
        // indice_vicino.resize(0);

        // cloud_proc->points.at(count).x = sum_point.x();
        // cloud_proc->points.at(count).y = sum_point.y();
        // cloud_proc->points.at(count).z = sum_point.z();
        // count++;

    }

    cloud->width = cloud->points.size();
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/prova.pcd", *cloud);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola2_proc.pcd", *cloud);

    return 0;
}