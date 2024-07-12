#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

void icp(std::string source_path, std::string target_path, std::string dest_path, std::string forma, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
    // Read two PCD files from disk.
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_path, *sourceCloud) != 0) { return; }
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_path, *targetCloud) != 0) { return; }
    Eigen::Matrix4f T;
    

    // ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(sourceCloud);
	icp.setInputTarget(targetCloud);

	icp.align(*tf_cloud);
	if(icp.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << forma + ": " << "The score is " << icp.getFitnessScore() << std::endl << std::endl;
		// std::cout << "Transformation matrix:" << std::endl << std::endl;
        T = icp.getFinalTransformation();
		// std::cout << T << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

    *transformed_cloud = *sourceCloud;
    Eigen::Vector4f punto, punto_tf;
    
    for(int i = 0; i < sourceCloud->points.size(); i++)
    {
        punto.x() = sourceCloud->points.at(i).x;
        punto.y() = sourceCloud->points.at(i).y;
        punto.z() = sourceCloud->points.at(i).z;
        punto.w() = 1;
        punto_tf = T*punto;
        transformed_cloud->points.at(i).x = punto_tf.x();
        transformed_cloud->points.at(i).y = punto_tf.y();
        transformed_cloud->points.at(i).z = punto_tf.z();
    }

    std::string path = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_" + forma + "2.pcd";
    ros::Rate loop_rate(20);
    pcl::io::savePCDFile(dest_path, *transformed_cloud);
    pcl::visualization::CloudViewer viewer("Align");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cluster) != 0) { return; }
	viewer.showCloud(transformed_cloud, "tattile");
    viewer.showCloud(targetCloud, "visuale");
	while (!viewer.wasStopped())
	{
        loop_rate.sleep();
		// Do nothing but wait.
	}

}

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "icp");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    // Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string path_spirale_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_spline.pcd";
    std::string path_cerchio_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_cerchio2_spline.pcd";
    std::string path_parabola_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_parabola2_spline.pcd";
    std::string path_retta_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_retta2_spline.pcd";

    std::string path_spirale_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale2_spline.pcd";
    std::string path_cerchio_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_cerchio2_spline.pcd";
    std::string path_parabola_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_parabola2_spline.pcd";
    std::string path_retta_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta2_spline.pcd";

    std::string path_spirale_tf = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_spirale2_icp.pcd";
    std::string path_cerchio_tf = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_cerchio2_icp.pcd";
    std::string path_parabola_tf = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_parabola2_icp.pcd";
    std::string path_retta_tf = "/home/workstation2/ws_cross_modal/bags/PCL_centr2_retta2_icp.pcd";

    std::string spirale = "spirale";
    std::string cerchio = "cerchio";
    std::string parabola = "parabola";
    std::string retta = "retta";

    icp(path_spirale_tattile, path_spirale_visuale, path_spirale_tf, spirale, transformed_cloud);
    icp(path_cerchio_tattile, path_cerchio_visuale, path_cerchio_tf, cerchio, transformed_cloud);
    icp(path_parabola_tattile, path_parabola_visuale, path_parabola_tf, parabola, transformed_cloud);
    icp(path_retta_tattile, path_retta_visuale, path_retta_tf, retta, transformed_cloud);

    return 0;

}