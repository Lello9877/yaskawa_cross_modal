#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef Eigen::Spline<float, 3> Spline3d;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "interpolation");
    ros::NodeHandle nh;

    // Prelevo i punti di via dalla point cloud tattile
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInterpolated(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud) != 0) { return -1; }
    std::vector<Eigen::Vector3d> waypoints;
    
    for(int i = 0; i < cloud->points.size(); i++)
    {   
        Eigen::Vector3d punto(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
        waypoints.push_back(punto);
    }

    std::vector<Eigen::VectorXf> waypoints;
    Eigen::Vector3f po1(2,3,4);
    Eigen::Vector3f po2(2,5,4);
    Eigen::Vector3f po3(2,8,9);
    Eigen::Vector3f po4(2,8,23);
    waypoints.push_back(po1);
    waypoints.push_back(po2);
    waypoints.push_back(po3);
    waypoints.push_back(po4);

    // The degree of the interpolating spline needs to be one less than the number of points
    // that are fitted to the spline.
    Eigen::MatrixXf points(3, waypoints.size());
    int row_index = 0;
    for(auto const way_point : waypoints){
        points.col(row_index) << way_point[0], way_point[1], way_point[2];
        row_index++;
    }
    Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
    float time_ = 0;
    for(int i=0; i<20; i++){
        time_ += 1.0/(20*1.0);
        Eigen::VectorXf values = spline(time_);
        std::cout << values << std::endl;
    }
    return 0;
}