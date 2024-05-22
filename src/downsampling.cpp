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

void downsampling(std::string path, int num_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(path, *cloud_to_filter);
    pcl::RandomSample<pcl::PointXYZRGB> sampler;
    sampler.setInputCloud(cloud_to_filter);
    sampler.setSample(num_points);
    sampler.filter(*cloud_filtered);
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

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd", *cloud);
    // // std::cout << *cloud << std::endl;
    // // Create a KD-Tree
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)

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


    // // Upsampling tattile retta
    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    // mls.setComputeNormals(true);
    // mls.setInputCloud(cloud);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    // //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.02);
    // mls.setUpsamplingStepSize(0.01);
    // mls.setPointDensity(100000);
    // mls.setPolynomialOrder(2);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.03);
    // mls.process(mls_points);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", mls_points);

    // // Filtraggio VoxelGrid tattile retta
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_mls.pcd", *cloud_to_filter);
    // pcl::VoxelGrid<pcl::PointXYZ> filter;
	// filter.setInputCloud(cloud_to_filter);
    // filter.setLeafSize(0.0005, 0.0, 0.0);
    // filter.filter(*filteredCloud);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_filtered.pcd", *filteredCloud);

    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    // double dec = 0;
    // /*for(int i = 0; i < 2; i++)*/ {
    //     filter.setInputCloud(cloud);
    //     // Object for searching.
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    //     filter.setSearchMethod(kdtree);
    //     // Use all neighbors in a radius of 3cm.
    //     filter.setSearchRadius(0.1);
    //     // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    //     // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
    //     //filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    //     filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    //     // Radius around each point, where the local plane will be sampled.
    //     filter.setUpsamplingRadius(0.03-dec);
    //     filter.setPointDensity(300);
    //     // Sampling step size. Bigger values will yield less (if any) new points.
    //     filter.setUpsamplingStepSize(0.01);
    //     filter.setPolynomialFit(true);
    //     filter.setPolynomialOrder(2);
    //     filter.process(*filteredCloud);
    //     std::cout << *filteredCloud << std::endl;
    //     pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_mls.pcd", *filteredCloud);
    //     pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_mls.pcd", *cloud);
    //     dec = dec + 0.00;
    // }

    // // Filtraggio VoxelGrid tattile spirale
    // pcl::io::loadPCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_mls.pcd", *cloud_to_filter);
    // pcl::VoxelGrid<pcl::PointXYZ> voxel;
	// voxel.setInputCloud(cloud_to_filter);
    // voxel.setLeafSize(0.005, 0.005, 0.005);
    // voxel.filter(*filteredCloud);
    // std::cout << *filteredCloud << std::endl;
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_filtered.pcd", *filteredCloud);


    // // Segmentazione visuale spirale
    // {
    //     segmentazione();
    // }

    // Downsampling delle point cloud visuali
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    int num_points = 0;
    std::string path_spirale_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd";
    std::string path_cerchio_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio_spline.pcd";
    std::string path_parabola_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola_spline.pcd";
    std::string path_retta_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_spline.pcd";
    std::string path_spirale_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_filtered.pcd";
    std::string path_cerchio_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_filtered.pcd";
    std::string path_parabola_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola_filtered.pcd";
    std::string path_retta_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_filtered.pcd";

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_spirale_tattile, *cloud) != 0) { return -1; }
    num_points = cloud->points.size();
    std::cout << "Punti Spirale: " << num_points << std::endl;
    downsampling(path_spirale_visuale, num_points, cloud_downsampled);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_down.pcd", *cloud_downsampled);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_cerchio_tattile, *cloud) != 0) { return -1; }
    num_points = cloud->points.size();
    std::cout << "Punti Cerchio: " << num_points << std::endl;
    downsampling(path_cerchio_visuale, num_points, cloud_downsampled);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio_down.pcd", *cloud_downsampled);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_parabola_tattile, *cloud) != 0) { return -1; }
    num_points = cloud->points.size();
    std::cout << "Punti Parabola: " << num_points << std::endl;
    downsampling(path_parabola_visuale, num_points, cloud_downsampled);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola_down.pcd", *cloud_downsampled);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_retta_tattile, *cloud) != 0) { return -1; }
    num_points = cloud->points.size();
    std::cout << "Punti Retta: " << num_points << std::endl;
    downsampling(path_retta_visuale, num_points, cloud_downsampled);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_down.pcd", *cloud_downsampled);
    
    return 0;
}