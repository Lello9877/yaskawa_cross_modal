#include <ros/ros.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include "yaskawa_cross_modal/VisualAction.h"
#include "yaskawa_cross_modal/utility.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


sensor_msgs::PointCloud2 nuvola;
geometry_msgs::PoseStamped pose_base_cam;

// Callbacks
void camera_cb(const sensor_msgs::PointCloud2Ptr &msg) { nuvola = *msg; }
void fkine_cb(const geometry_msgs::PoseStampedPtr &msg) { pose_base_cam = *msg; }

class VisualActionServer
{
public:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<yaskawa_cross_modal::VisualAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    // yaskawa_cross_modal::task_tactileAction::feedback_;
    yaskawa_cross_modal::VisualResult result_;

    VisualActionServer(std::string name) : as_(nh, name, boost::bind(&VisualActionServer::executeCB, this, _1), false)
    {
        as_.start();
    }

    ~VisualActionServer() = default;

    void executeCB(const yaskawa_cross_modal::VisualGoalConstPtr &goal)
    {
        ros::Subscriber sub_camera = nh.subscribe("/camera/depth/color/points", 10, camera_cb);
        ros::Subscriber sub_fkine = nh.subscribe("/motoman/clik/fkine", 10, fkine_cb);
        ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/visual_cloud", 10);
        ros::Publisher pub_cloud_bag = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_camera", 10);
        ros::Rate loop_rate(30);
        sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
        robot.waitForServers();
        bool success = true;
        
        std::cout << std::endl << "VISUAL EXPLORATION" << std::endl;

        geometry_msgs::Pose end_effector;
        end_effector.position.x = 0.102695;
        end_effector.position.y = 0.016692;
        end_effector.position.z = -0.00721836;
        Eigen::Quaterniond Quat;
        Quat.x() = -0.133105;
        Quat.y() = -0.133105;
        Quat.z() = 0.694466;
        Quat.w() = 0.694466;
        Quat.normalize();
        end_effector.orientation.x = Quat.x();
        end_effector.orientation.y = Quat.y();
        end_effector.orientation.z = Quat.z();
        end_effector.orientation.w = Quat.w();
        robot.clik_.set_end_effector(end_effector);

        // Configurazioni per la cattura della point cloud
        const std::vector<double> q_home = goal->q0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_media(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2 visual_cloud;
        int count = 0;

        if(askContinue("Home")) 
            robot.goTo(q_home, ros::Duration(20.0));
        ros::spinOnce();

        // pcl::fromROSMsg(nuvola, *cloud_media);
        // for(int i = 0; i < cloud_media->points.size(); i++)
        // {
        //     cloud_media->points.at(i).x = 0.0;
        //     cloud_media->points.at(i).y = 0.0;
        //     cloud_media->points.at(i).z = 0.0;
        // }

        ros::Time t0 = ros::Time::now();
        ros::Duration t = ros::Time::now() - t0;

        if(askContinue("Iniziare la cattura della point Cloud?")) 
        {

            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
            }

            // t = ros::Time::now() - t0;

            // tf2_ros::Buffer tfBuffer;
            // tf2_ros::TransformListener tfListener(tfBuffer);
            // geometry_msgs::TransformStamped transform;

            // try {
            //     transform = tfBuffer.lookupTransform("tool0", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); 
            // }
            // catch (tf2::TransformException &ex) {
            //     ROS_WARN("%s",ex.what());
            //     ros::Duration(1.0).sleep();
            // }

            // std::cout << transform;
            
            // try {
            //     transform = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); 
            // }
            // catch (tf2::TransformException &ex) {
            //     ROS_WARN("%s",ex.what());
            //     ros::Duration(1.0).sleep();
            // }

            //std::cout << transform << std::endl;

            Eigen::Vector3d o_base_cam;
            Eigen::Vector3d p_cam_cam;
            Eigen::Vector3d p_base_cam;
            Eigen::Matrix3d R_base_cam;
            Eigen::Quaterniond q;

            // o_base_cam.x() = transform.transform.translation.x;
            // o_base_cam.y() = transform.transform.translation.y;
            // o_base_cam.z() = transform.transform.translation.z;
            // q.w() = transform.transform.rotation.w;
            // q.x() = transform.transform.rotation.x;
            // q.y() = transform.transform.rotation.y;
            // q.z() = transform.transform.rotation.z;
            // q.normalize();
            // R_base_cam = q.toRotationMatrix();
            o_base_cam.x() = pose_base_cam.pose.position.x;
            o_base_cam.y() = pose_base_cam.pose.position.y;
            o_base_cam.z() = pose_base_cam.pose.position.z;
            q.w() = pose_base_cam.pose.orientation.w;
            q.x() = pose_base_cam.pose.orientation.x;
            q.y() = pose_base_cam.pose.orientation.y;
            q.z() = pose_base_cam.pose.orientation.z;
            q.normalize();
            R_base_cam = q.toRotationMatrix();

            // sensor_msgs::convertPointCloud2ToPointCloud(nuvola, cloud_temp);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            sensor_msgs::PointCloud2 cloud2;
            pcl::fromROSMsg(nuvola, *cloud_temp);
            for(int i = 0; i < cloud_temp->points.size(); i++) 
            {
                p_cam_cam.x() = cloud_temp->points.at(i).x;
                p_cam_cam.y() = cloud_temp->points.at(i).y;
                p_cam_cam.z() = cloud_temp->points.at(i).z;
                p_base_cam = o_base_cam + R_base_cam * p_cam_cam;
                cloud_temp->points.at(i).x = p_base_cam.x();
                cloud_temp->points.at(i).y = p_base_cam.y();
                cloud_temp->points.at(i).z = p_base_cam.z();
            }

            pcl::io::savePCDFile("/home/workstation2/pcl_catturata.pcd", *cloud_temp);

            // int dim_media = cloud_media->points.size();
            // int dim_temp = cloud_temp->points.size();
            // int actual_dim;

            // std::cout << "Iterazione " << count << std::endl;

            // std::cout << "dim_media PRIMA: " << dim_media << std::endl;
            // std::cout << "dim_temp PRIMA: " << dim_temp << std::endl;

            // if(dim_media == dim_temp) {}
            // else if(dim_media < dim_temp)
            // {
            //     cloud_media->points.resize(dim_temp);
            // }
            // else
            // {
            //     cloud_temp->points.resize(dim_media);
            // }

            // std::cout << "dim_media DOPO: " << dim_media << std::endl;
            // std::cout << "dim_temp DOPO: " << dim_temp << std::endl;

            // for(int i = 0; i < cloud_media->points.size(); i++)
            // {
            //     cloud_media->points.at(i).x += cloud_temp->points.at(i).x;
            //     cloud_media->points.at(i).y += cloud_temp->points.at(i).y;
            //     cloud_media->points.at(i).z += cloud_temp->points.at(i).z;
            // }

            // count++;
            // cloud_media->width = cloud_media->points.size();
            // cloud_media->height = 1;
            pcl::toROSMsg(*cloud_temp, cloud2);
            visual_cloud = cloud2;
            // cloud2.header.frame_id = "base_link";
            // cloud2.header.stamp = ros::Time::now();
            // pub_cloud2.publish(cloud2);
            ros::spinOnce();

            if(success)
            {
                visual_cloud.header.frame_id = "base_link";
                visual_cloud.header.stamp = ros::Time::now();
                pub_cloud2.publish(visual_cloud);
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }

            // for(int i = 0; i < cloud_media->points.size(); i++)
            // {
            //     cloud_media->points.at(i).x /= count;
            //     cloud_media->points.at(i).y /= count;
            //     cloud_media->points.at(i).z /= count;
            // }

            // cloud_media->width = cloud_media->points.size();
            // cloud_media->height = 1;
            // pcl::toROSMsg(*cloud_media, cloud2);
            // cloud2.header.frame_id = "base_link";
            // cloud2.header.stamp = ros::Time::now();
            // pub_cloud2.publish(cloud2);            
            loop_rate.sleep();
        }
        else {}
    }

};

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "visual_exploration");
    VisualActionServer server("visual_action");
    ros::spin();

    return 0;
}