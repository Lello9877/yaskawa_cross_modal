#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "yaskawa_cross_modal/utility.h"
#include <thread>

using namespace std::chrono_literals;

double cluster_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu2)
{
    double distance = 500.0;
    double temp_distance = 0.0;
    for(int i = 0; i < clu1->points.size(); i++)
    {
        for(int j = 0; j < clu2->points.size(); j++)
        {
            temp_distance = euclideanDistance(clu1->points.at(i).x, clu1->points.at(i).y, clu1->points.at(i).z, clu2->points.at(j).x, clu2->points.at(j).y, clu2->points.at(j).z);
            if(temp_distance < distance)
                distance = temp_distance;
            if(distance < 0.008) 
                return distance;
        }
    }
    return distance;
}

void segmentazione(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    pcl::io::loadPCDFile(path, *cloud_rgb);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_rgb);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud_rgb);
	clustering.setSearchMethod(kdtree);
    clustering.setMinClusterSize(600);
    clustering.setDistanceThreshold(10);
    clustering.setPointColorThreshold(6);
    clustering.setRegionColorThreshold(5);
    std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

    // For every cluster...
	int currentClusterCavo = 0;
    int count_tavolo = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_vector;
	for(std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud_rgb->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = cluster;
        // pcl::visualization::CloudViewer viewer("Cluster viewer");
        // viewer.showCloud(colored_cloud);

        // while(!viewer.wasStopped ())
        //     std::this_thread::sleep_for(100us);

        int count = 0;

        for(int j = 0; j < cluster->points.size(); j++)
        {
            if(cluster->points.at(j).r < 195 && cluster->points.at(j).r > 130)
                if(cluster->points.at(j).g < 195 && cluster->points.at(j).g > 130)
                    if(cluster->points.at(j).b < 195 && cluster->points.at(j).b > 130)
                        count++;
        }

        // std::cout << count << std::endl;

        double media = static_cast<double>(count)/cluster->points.size();
        std::cout << media << std::endl;
        if(media > 0.70)
        {
            std::cout << "TAVOLO" << std::endl;
            count_tavolo++;
        }
        else
        {    
            std::cout << "NON TAVOLO" << std::endl;
            currentClusterCavo++;
            cluster_vector.push_back(cluster);
        }
	}

    double distance = -1.0;
    std::vector<int> indice;
    // indice.resize(cluster_vector.size());
    std::vector<int> indice_occlusione;
    bool indice_presente_1 = false;
    bool indice_presente_2 = false;
    std::cout << "Dimensione del vettore cluster NON TAVOLO " << cluster_vector.size() << std::endl;
    // indice.resize(cluster_vector.size());
    
    for(int i = 0; i < cluster_vector.size() - 1; i++)
    {
        for(int j = i + 1; j < cluster_vector.size(); j++)
        {
            distance = cluster_distance(cluster_vector.at(i), cluster_vector.at(j));
            if(distance > 0.05)
            {
                if(indice.size() == 0)
                {
                    indice.push_back(i);
                    indice.push_back(j);
                }
                else
                {
                    for(int k = 0; k < indice.size(); k++)
                    {
                        if(indice.at(k) == i)
                        {
                            indice_presente_1 = true;
                            break;
                        }
                    }
                    for(int k = 0; k < indice.size(); k++)
                    {
                        if(indice.at(k) == j)
                        {
                            indice_presente_2 = true;
                            break;
                        }
                    }

                    if(indice_presente_1 == true && indice_presente_2 == false)
                        indice.push_back(j);
                    else if(indice_presente_1 == false && indice_presente_2 == true)
                        indice.push_back(i);
                    else if(indice_presente_1 == false && indice_presente_2 == false)
                    {
                        indice.push_back(i);
                        indice.push_back(j);
                    }
                    else {}
                }
            }
        }
    }

    bool presente;
    std::cout << "Gli indici dei cavi sono " << indice.size() << std::endl;
    for(int i = 0; i < indice.size(); i++)
        std::cout << "Valore " << i << ": " << indice.at(i) << std::endl;
    for(int i = 0; i < cluster_vector.size(); i++)
    {
        presente = false;
        for(int j = 0; j < indice.size(); j++)
        {
            if(i == indice.at(j))
            {
                presente = true;
                break;
            }
        }
        if(!presente)
        {
            std::cout << "Il cluster di indice " << i << " è un'occlusione" << std::endl;
            indice_occlusione.push_back(i);
        }
    }

    for(int i = 0; i < indice_occlusione.size(); i++)
    {
        std::string path_occlusione = "/home/workstation2/ws_cross_modal/occlusione" + boost::to_string(indice_occlusione.at(i)) + ".pcd";
        pcl::io::savePCDFile(path_occlusione, *cluster_vector.at(indice_occlusione.at(i)));
    }

    for(int i = 0; i < indice.size(); i++)
    {
        std::string path_cavo = "/home/workstation2/ws_cross_modal/cavo" + boost::to_string(indice.at(i)) + ".pcd";
        pcl::io::savePCDFile(path_cavo, *cluster_vector.at(indice.at(i)));
    }

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = clustering.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);

    // while (!viewer.wasStopped ())
    //     std::this_thread::sleep_for(100us);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;

    // for(int i = 19; i < 30; i++)
    // {
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     std::string path = "/home/workstation2/ws_cross_modal/bags/" + boost::to_string(i) + "_pre.pcd";
    //     segmentazione(path, cloud);
    // }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string path = "/home/workstation2/ws_cross_modal/pcl_occlusa_pre2.pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) { return -1; }
    segmentazione(path, cloud);

    return 0;
}