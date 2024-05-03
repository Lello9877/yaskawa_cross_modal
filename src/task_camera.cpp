#include <ros/ros.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Dense>
#define foreach BOOST_FOREACH

sensor_msgs::PointCloud2 nuvola;
geometry_msgs::Pose pose_base_cam;

// void read_from_bag() {
   
//     rosbag::Bag bag_point;
//     std::vector<std::string> topics;
//     topics.push_back(std::string("/camera/depth/color/points"));
    
//     bag_point.open("/home/workstation2/ws_cross_modal/bags/PCL_realsense.bag", rosbag::bagmode::Read);
//     rosbag::View view(bag_point, rosbag::TopicQuery(topics.at(0)));
    
//     foreach(rosbag::MessageInstance const m, view)
//     {
//         sensor_msgs::PointCloud2ConstPtr pcl = m.instantiate<sensor_msgs::PointCloud2>();
//         if(pcl != NULL) {
//             nuvola = *pcl;
//             break;
//         }

//     }

//     bag_point.close();

// }


void camera_cb(const sensor_msgs::PointCloud2Ptr &msg) { nuvola = *msg; }
void fkine_cb(const geometry_msgs::PosePtr &msg) { pose_base_cam = *msg; }

int main(int argc, char **argv) 
{

    ros::init(argc,argv,"task_camera");
    ros::NodeHandle nh;
    ros::Subscriber sub_camera = nh.subscribe("/camera/depth/color/points", 1, camera_cb);
    ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_base", 1);
    ros::Publisher pub_cloud_bag = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_camera", 1);
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transform;
    ros::Rate loop_rate(50.0);

    // try {
    //     transform = tfBuffer.lookupTransform("tool0", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); 
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }

    // std::cout << transform;

    Eigen::Vector3d o_base_cam;
    Eigen::Vector3d p_cam_cam;
    Eigen::Vector3d p_base_cam;
    Eigen::Matrix3d R_base_cam;
    Eigen::Quaterniond q;
    //read_from_bag();
    
    geometry_msgs::Pose end_effector;
    end_effector.position.x = 0.102695;
    end_effector.position.y = 0.016692;
    end_effector.position.z = -0.00721836;
    Eigen::Quaterniond Quat(-3.00045e-08, -0.561361, -5.7508e-09, -0.827571);
    Quat.normalize();
    end_effector.orientation.x = Quat.x();
    end_effector.orientation.y = Quat.y();
    end_effector.orientation.z = Quat.z();
    end_effector.orientation.w = Quat.w();
    robot.clik_.set_end_effector(end_effector);

    int count = 0;
    while(ros::ok()) 
    {
        
        try {
            transform = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); 
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        o_base_cam.x() = transform.transform.translation.x;
        o_base_cam.y() = transform.transform.translation.y;
        o_base_cam.z() = transform.transform.translation.z;
        q.w() = transform.transform.rotation.w;
        q.x() = transform.transform.rotation.x;
        q.y() = transform.transform.rotation.y;
        q.z() = transform.transform.rotation.z;
        q.normalize();
        R_base_cam = q.toRotationMatrix();
        // o_base_cam.x() = pose_base_cam.position.x;
        // o_base_cam.y() = pose_base_cam.position.y;
        // o_base_cam.z() = pose_base_cam.position.z;
        // q.w() = pose_base_cam.orientation.w;
        // q.x() = pose_base_cam.orientation.x;
        // q.y() = pose_base_cam.orientation.y;
        // q.z() = pose_base_cam.orientation.z;
        // q.normalize();
        // R_base_cam = q.toRotationMatrix();

        sensor_msgs::PointCloud cloud_temp;
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::convertPointCloud2ToPointCloud(nuvola, cloud_temp);

        for(int i = 0; i < cloud_temp.points.size(); i++) {
            p_cam_cam.x() = cloud_temp.points.at(i).x;
            p_cam_cam.y() = cloud_temp.points.at(i).y;
            p_cam_cam.z() = cloud_temp.points.at(i).z;
            p_base_cam = o_base_cam + R_base_cam * p_cam_cam;
            cloud_temp.points.at(i).x = p_base_cam.x();
            cloud_temp.points.at(i).y = p_base_cam.y();
            cloud_temp.points.at(i).z = p_base_cam.z();
        }

        sensor_msgs::convertPointCloudToPointCloud2(cloud_temp, cloud2);
        cloud2.header.frame_id = "base_link";
        cloud2.header.stamp = ros::Time::now();
        pub_cloud2.publish(cloud2);
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}