#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types_conversion.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/uniform_sampling.h>
#define foreach BOOST_FOREACH

template <typename T>
void bag_write(std::string path, std::string topic, T data) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write(topic,ros::Time::now(), data);
    bag.close();

}

template <typename T>
T bag_read(std::string path, std::string topic) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    T val;

    foreach(rosbag::MessageInstance const m, view)
    {
        boost::shared_ptr<T> pcl = m.instantiate<T>();
        if(pcl != NULL) {
            val = *pcl;
        }
    }

    bag.close();
    return val;

}

template <typename T>
int min(std::vector<T> vec) 
{
    int indice_minimo = 0;
    T valore_minimo = vec.at(0);
    for(int l = 0; l < vec.size(); l++)
    {
        if(vec.at(l) <= valore_minimo)
        {
            valore_minimo = vec.at(l);
            indice_minimo = l;
        }
    }
    return indice_minimo;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "downsampling");
    ros::NodeHandle nh;

    // Downsampling delle point cloud visuali
    int num_points = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path_spirale_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd";
    std::string path_cerchio_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio_spline.pcd";
    std::string path_parabola_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola_spline.pcd";
    std::string path_retta_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_spline.pcd";
    std::string path_spirale_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_filtered.pcd";
    std::string path_cerchio_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_filtered.pcd";
    std::string path_parabola_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola_filtered.pcd";
    std::string path_retta_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_filtered.pcd";
    std::string path = "/home/workstation2/ws_cross_modal/cerchio_occluso3.pcd";
    std::string path_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2.pcd";

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) { return -1; }
    
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);

    int divx, divy;
    double dmin = 0.015;
    double div, sumx, sumy, sumz;

    div = (x_max-x_min)/dmin;
    if(div < 1) divx = 1;
    else divx = ceil(div); /*divx = floor(div)*/

    div = (y_max-y_min)/dmin;
    if(div < 1) divy = 1;
    else divy = ceil(div);
    
    std::cout << "divx: " << divx << std::endl;
    std::cout << "divy: " << divy << std::endl;

    pcl::PointXYZ centroide;

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

    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/cerchio_occluso3_down.pcd", *cloud_voxel);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_grid.pcd", *cloud_voxel);

    return 0;
}