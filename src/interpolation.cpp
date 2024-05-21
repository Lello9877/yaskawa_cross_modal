#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


typedef Eigen::Spline<float, 3> Spline3d;

void spline(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInterpolated, int num_points) 
{
    std::vector<Eigen::Vector3d> waypoints;
    for(int i = 0; i < cloud->points.size(); i++)
    {   
        Eigen::Vector3d punto(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
        waypoints.push_back(punto);
    }

    Eigen::MatrixXf points(3, waypoints.size());
    int row_index = 0;
    for(auto const way_point : waypoints)
    {
        points.col(row_index) << way_point[0], way_point[1], way_point[2];
        row_index++;
    }

    Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 3);
    float time_ = 0;
    *cloudInterpolated = *cloud;
    cloudInterpolated->points.resize(cloud->points.size() + num_points);
    cloudInterpolated->width = cloud->points.size() + num_points;

    for(int i = cloud->points.size(); i < cloud->points.size() + num_points; i++)
    {
        time_ += 1.0/(num_points*1.0);
        Eigen::VectorXf values = spline(time_);
        //std::cout << values << std::endl << std::endl;
        cloudInterpolated->points.at(i).x = values.x();
        cloudInterpolated->points.at(i).y = values.y();
        cloudInterpolated->points.at(i).z = values.z();
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "interpolation");
    ros::NodeHandle nh;

    // Prelevo i punti di via dalla point cloud tattile
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInterpolated(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr prova(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd", *cloud) != 0) { return -1; }
    
    
    // Indici Ordinati Spirale: 6, 11, 14, 18, 19, 17, 15, 12, 9, 7, 4, 2, 1, 0, 3, 5, 8, 10, 13, 16, 20, 21, 22
    // Indici Ordinati Cerchio: 13, 14, 15, 18, 17, 16, 12, 11, 10, 8, 6, 4, 2, 1, 0, 3, 5, 7, 9
    // Indici Ordinati Parabola: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 11
    int indice_spirale[cloud->points.size()] = {6, 11, 14, 18, 19, 17, 15, 12, 9, 7, 4, 2, 1, 0, 3, 5, 8, 10, 13, 16, 20, 21, 22};
    *sortedCloud = *cloud;
    for(int i = 0; i < cloud->points.size(); i++)
    {   
        sortedCloud->points.at(i).x = cloud->points.at(indice_spirale[i]).x;
        sortedCloud->points.at(i).y = cloud->points.at(indice_spirale[i]).y;
        sortedCloud->points.at(i).z = cloud->points.at(indice_spirale[i]).z;
    }

    int num_points = 400;
    spline(sortedCloud, cloudInterpolated, num_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd", *cloudInterpolated);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio.pcd", *cloud) != 0) { return -1; }
    int indice_cerchio[cloud->points.size()] = {13, 14, 15, 18, 17, 16, 12, 11, 10, 8, 6, 4, 2, 1, 0, 3, 5, 7, 9};
    *sortedCloud = *cloud;
    for(int i = 0; i < cloud->points.size(); i++)
    {   
        sortedCloud->points.at(i).x = cloud->points.at(indice_cerchio[i]).x;
        sortedCloud->points.at(i).y = cloud->points.at(indice_cerchio[i]).y;
        sortedCloud->points.at(i).z = cloud->points.at(indice_cerchio[i]).z;
    }

    spline(sortedCloud, cloudInterpolated, num_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio_spline.pcd", *cloudInterpolated);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola.pcd", *cloud) != 0) { return -1; }
    int indice_parabola[cloud->points.size()] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 10};
    *sortedCloud = *cloud;
    for(int i = 0; i < cloud->points.size(); i++)
    {   
        sortedCloud->points.at(i).x = cloud->points.at(indice_parabola[i]).x;
        sortedCloud->points.at(i).y = cloud->points.at(indice_parabola[i]).y;
        sortedCloud->points.at(i).z = cloud->points.at(indice_parabola[i]).z;
    }

    spline(sortedCloud, cloudInterpolated, num_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola_spline.pcd", *cloudInterpolated);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud) != 0) { return -1; }
    spline(cloud, cloudInterpolated, num_points);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_spline.pcd", *cloudInterpolated);

    //pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd", *cloud);

    // int dim = 4;
    // prova->points.resize(dim);
    // prova->width = dim;
    // prova->height = 1;

    // for(int i = 0; i < dim; i++) {
    //     prova->points.at(i).x = cloud->points.at(i).x;
    //     prova->points.at(i).y = cloud->points.at(i).y;
    //     prova->points.at(i).z = cloud->points.at(i).z;
    // }
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova.pcd", *prova);

    // std::cout << "Width: " << cloud->width << std::endl;
    // std::cout << "height: " << cloud->height << std::endl;
    return 0;
}