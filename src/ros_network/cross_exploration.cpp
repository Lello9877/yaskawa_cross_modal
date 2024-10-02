#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <actionlib/client/simple_action_client.h>
#include "yaskawa_cross_modal/TactileAction.h"
#include "yaskawa_cross_modal/utility.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
void occlusion_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *visual_cloud);
}

void downsampled_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 nuvola = *msg;
    pcl::fromROSMsg(nuvola, *downsampled_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cross_exploration");
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster = nh.subscribe("/occlusion", 10, occlusion_cb);
    ros::Subscriber sub_cloud_downsampled = nh.subscribe("/cloud_downsampled", 10, downsampled_cb);
    actionlib::SimpleActionClient<yaskawa_cross_modal::TactileAction> ac("tactile_action", true);
    ros::Rate loop_rate(30);
    double centr_x = 0.0, centr_y = 0.0, centr_z = 0.0;

    while(ros::ok())
    {
        ros::spinOnce();
        if(visual_cloud->points.size() != 0)
        {
            std::cout << std::endl << "CROSS EXPLORATION" << std::endl;
            // for(int i = 0; i < visual_cloud->points.size(); i++)
            // {
            //     centr_x += visual_cloud->points.at(i).x;
            //     centr_y += visual_cloud->points.at(i).y;
            //     centr_z += visual_cloud->points.at(i).z;
            // }

            // Ricerca x e y per definire la griglia
            double x_min, y_min, x_max, y_max;
            x_min = visual_cloud->points.at(0).x;
            x_max = visual_cloud->points.at(0).x;
            y_max = visual_cloud->points.at(0).y;
            y_min = visual_cloud->points.at(0).y;

            for(int i = 1; i < visual_cloud->points.size(); i++)
            {
                if(visual_cloud->points.at(i).x <= x_min)
                    x_min = visual_cloud->points.at(i).x;
                if(visual_cloud->points.at(i).x >= x_max)
                    x_max = visual_cloud->points.at(i).x;            
                if(visual_cloud->points.at(i).y <= y_min)
                    y_min = visual_cloud->points.at(i).y;
                if(visual_cloud->points.at(i).y >= y_max)
                    y_max = visual_cloud->points.at(i).y;
            }

            std::cout << "Coordinata x minima: " << x_min << std::endl;
            std::cout << "Coordinata x massima: " << x_max << std::endl;
            std::cout << "Coordinata y minima: " << y_min << std::endl;
            std::cout << "Coordinata y massima: " << y_max << std::endl;

            int rows = 6, cols = 2;
            double dmin = 0.015;
            double z_start = 0.270;
            Eigen::Quaterniond Q(0, 0.7071, 0.7071, 0);
            Q.normalize();
            geometry_msgs::Pose start;
            start.position.x = x_min;
            start.position.y = y_max;
            start.position.z = z_start;
            start.orientation.w = Q.w();
            start.orientation.x = Q.x();
            start.orientation.y = Q.y();
            start.orientation.z = Q.z();
            
            yaskawa_cross_modal::TactileGoal goal;
            goal.rows = rows;
            goal.cols = cols;
            goal.dmin = dmin;
            goal.x_max = x_max;
            goal.x_min = x_min;
            goal.y_max = y_max;
            goal.y_min = y_min;
            goal.start = start;

            if(askContinue("Mando il goal per fare l'esplorazione tattile?"))
            {
                ac.sendGoalAndWait(goal);
                std::cout << "L'esplorazione è terminata" << std::endl;
                visual_cloud->points.clear();
                visual_cloud->points.resize(0);
            }
            else {break;}
        }
        loop_rate.sleep();
    }
    return 0;
}