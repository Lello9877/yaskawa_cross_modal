#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "icp");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    // Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read two PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *sourceCloud) != 0) { return -1; }
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio.pcd", *targetCloud) != 0) { return -1; }
    Eigen::Matrix4f T;

	// ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(sourceCloud);
	icp.setInputTarget(targetCloud);

	icp.align(*finalCloud);
	if (icp.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << icp.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
        T = icp.getFinalTransformation();
		std::cout << T << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_transformed = *sourceCloud;
    Eigen::Vector4f punto, punto_tf;
    
    for(int i = 0; i < sourceCloud->points.size(); i++)
    {
        punto.x() = sourceCloud->points.at(i).x;
        punto.y() = sourceCloud->points.at(i).y;
        punto.z() = sourceCloud->points.at(i).z;
        punto.w() = 1;
        punto_tf = T*punto;
        cloud_transformed->points.at(i).x = punto_tf.x();
        cloud_transformed->points.at(i).y = punto_tf.y();
        cloud_transformed->points.at(i).z = punto_tf.z();
    }

    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/cloud_tf.pcd", *cloud_transformed);
    pcl::visualization::CloudViewer viewer("Align");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale.pcd", *cluster) != 0) { return -1; }
	viewer.showCloud(cloud_transformed, "tattile");
    viewer.showCloud(targetCloud, "visuale");
	while (!viewer.wasStopped())
	{
        loop_rate.sleep();
		// Do nothing but wait.
	}

    return 0;

}