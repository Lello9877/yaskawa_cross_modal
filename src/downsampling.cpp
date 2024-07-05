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

void segmentazione(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    pcl::io::loadPCDFile(path, *cloud_rgb);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_rgb);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud_rgb);
	clustering.setSearchMethod(kdtree);
    clustering.setMinClusterSize(100);
    clustering.setDistanceThreshold(10);
    clustering.setPointColorThreshold(6);
    clustering.setRegionColorThreshold(5);
    std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

    // For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud_rgb->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "/home/workstation2/ws_cross_modal/bags/cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFile(fileName, *cluster);

		currentClusterNum++;
	}

    // // Codice per concatenare i cluster
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/cluster3.pcd", *clu);
    // *concat += *clu;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/cluster7.pcd", *clu);
    // *concat += *clu;
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_merge.pcd", *concat);

}

void downsampling(std::string path, int num_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(path, *cloud_to_filter);
    pcl::RandomSample<pcl::PointXYZ> sampler;
    sampler.setInputCloud(cloud_to_filter);
    sampler.setSample(num_points);
    sampler.filter(*cloud_filtered);
}

void uniformsampling(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud_to_filter) != 0) { return; }
    pcl::UniformSampling<pcl::PointXYZ> sampler;
    sampler.setInputCloud(cloud_to_filter);
    sampler.setRadiusSearch(0.01);
    sampler.filter(*cloud_downsampled);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "downsampling");
    ros::NodeHandle nh;

    // sensor_msgs::PointCloud2 PCL = bag_read<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_realsense_cerchio.bag", "/camera/depth/color/points");
    // bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio.bag", "/camera/depth/color/points", PCL);
    // sensor_msgs::PointCloud centr = bag_read<sensor_msgs::PointCloud>("/home/workstation2/ws_cross_modal/bags/PCL_centroide_spirale.bag", "/pcl2");
    // sensor_msgs::PointCloud2 centr2;
    // sensor_msgs::convertPointCloudToPointCloud2(centr, centr2);
    // bag_write<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.bag", "/pcl2", centr2);

    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd", *cloud);
    // // std::cout << *cloud << std::endl;
    // // Create a KD-Tree
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointNormal> mls_points;

    // Upsampling tattile spirale
    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    // mls.setComputeNormals(true);
    // mls.setInputCloud(cloud);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.03);
    // mls.setUpsamplingStepSize(0.01);
    // mls.setPointDensity(500);
    // mls.setPolynomialOrder(5);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.05);
    // mls.process(mls_points);
    // std::cout << mls_points << std::endl;
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_mls.pcd", mls_points);

    // // Segmentazione visuale spirale
    // {
    //     segmentazione();
    // }

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_spirale_tattile, *cloud) != 0) { return -1; }
    // num_points = cloud->points.size();
    // // std::cout << "Punti Spirale: " << num_points << std::endl;
    // downsampling(path_spirale_visuale, num_points, cloud_downsampled);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_down.pcd", *cloud_downsampled);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_cerchio_tattile, *cloud) != 0) { return -1; }
    // num_points = cloud->points.size();
    // // std::cout << "Punti Cerchio: " << num_points << std::endl;
    // downsampling(path_cerchio_visuale, num_points, cloud_downsampled);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_down.pcd", *cloud_downsampled);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_spirale_tattile, *cloud) != 0) { return -1; }
    // uniformsampling(path_spirale_visuale, cloud_downsampled);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_uniform.pcd", *cloud_downsampled);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_parabola_tattile, *cloud) != 0) { return -1; }
    // num_points = cloud->points.size();
    // std::cout << "Punti Parabola: " << num_points << std::endl;
    // downsampling(path_parabola_visuale, num_points, cloud_downsampled);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola_down.pcd", *cloud_downsampled);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_retta_tattile, *cloud) != 0) { return -1; }
    // num_points = cloud->points.size();
    // std::cout << "Punti Retta: " << num_points << std::endl;
    // downsampling(path_retta_visuale, num_points, cloud_downsampled);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_down.pcd", *cloud_downsampled);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
    // std::vector<int> inliers;
    // if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_down.pcd", *cloud) != 0) { return -1; }
    // for(int i = 0; i < cloud->points.size(); i++)
    //     cloud->points.at(i).z = 0;

    // // Filtraggio Voxel Grid per usare la Spline con la SPIRALE
    // pcl::VoxelGrid<pcl::PointXYZ> filter;
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_uniform.pcd", *cloud_to_filter);
	// filter.setInputCloud(cloud_to_filter);
    // filter.setLeafSize(0.07, 0.07, 0.07);
    // filter.filter(*filteredCloud);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_voxel.pcd", *filteredCloud);

    // // Filtraggio Voxel Grid per usare la Spline con il CERCHIO
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_filtered.pcd", *cloud_to_filter);
	// filter.setInputCloud(cloud_to_filter);
    // filter.setLeafSize(0.075, 0.075, 0.075);
    // filter.filter(*filteredCloud);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_voxel.pcd", *filteredCloud);

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
    std::string path = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_cen.pcd";
    std::string path_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_grid.pcd";

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_visuale, *cloud) != 0) { return -1; }
    
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

    // SPIRALE: divx = 6; divy = 8 OK
    // Assunzione: dmin = 0.04 (4 cm)
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

    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_grid.pcd", *cloud_voxel);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_grid.pcd", *cloud_voxel);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr prova(new pcl::PointCloud<pcl::PointXYZ>);
    // int num = 5;
    // prova->points.resize(num);
    // prova->width = num;
    // prova->height = 1;

    // for(int i = 0; i < num; i++)
    //     prova->points.at(i) = cloud_voxel->points.at(i);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova.pcd", *prova);

    return 0;
}