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
geometry_msgs::PoseStamped pose_base_cam;

// Callbacks
void camera_cb(const sensor_msgs::PointCloud2Ptr &msg) { nuvola = *msg; }
void fkine_cb(const geometry_msgs::PoseStampedPtr &msg) { pose_base_cam = *msg; }

// template <typename T>
// T bag_read(std::string path, std::string topic) {

//     rosbag::Bag bag;
//     bag.open(path, rosbag::bagmode::Read);
//     rosbag::View view(bag, rosbag::TopicQuery(topic));
//     T val;

//     foreach(rosbag::MessageInstance const m, view)
//     {
//         boost::shared_ptr<T> pcl = m.instantiate<T>();
//         if(pcl != NULL) {
//             val = *pcl;
//         }
//     }

//     bag.close();
//     return val;

// }

bool askContinue(const std::string &prompt = "")
{
    char ans;
    std::cout << prompt << " - Press y to continue [s to skip]: ";
    std::cin >> ans;

    if (ans == 'y' || ans == 'Y') return true;
    else if (ans == 's' || ans == 'S') return false;
    else return false;

    throw std::runtime_error("USER STOP!");
}

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "task_camera");
    ros::NodeHandle nh;
    ros::Subscriber sub_camera = nh.subscribe("/camera/depth/color/points", 1, camera_cb);
    ros::Subscriber sub_fkine = nh.subscribe("/motoman/clik/fkine", 1, fkine_cb);
    ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_base", 1);
    ros::Publisher pub_cloud_bag = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_camera", 1);
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

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

    ros::Rate loop_rate(50.0);
    Eigen::Vector3d o_base_cam;
    Eigen::Vector3d p_cam_cam;
    Eigen::Vector3d p_base_cam;
    Eigen::Matrix3d R_base_cam;
    Eigen::Quaterniond q;
    
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

    // Configurazioni delle catture
    // In ordine da come seguono: spirale, cerchio
    const std::vector<double> q_home = { -1.5594812631607056, -0.32599368691444397, 0.0, -0.13230204582214355, 0.031120063737034798, -1.3393893241882324, -0.022879714146256447 };
    const std::vector<double> q_retta = { -1.6433945894241333, -0.19776201248168945, 4.55637855338864e-05, 0.23550401628017426, 0.0029920218512415886, -1.5813273191452026, -0.006395920179784298 };
    const std::vector<double> q1 = { -1.6057740449905396, -0.3545166254043579, 0.0, -0.057136986404657364, -0.012833799235522747, -1.4666751623153687, 0.13985225558280945 };
    const std::vector<double> q_parabola = { -1.6755170822143555, -0.2627359628677368, -0.0002278189203934744, 0.14147554337978363, -0.010236663743853569, -1.5524402856826782, 0.023347707465291023 };
    const std::vector<double> q_spirale = {};

    if(askContinue("Home"))
        robot.goTo(q_home, ros::Duration(20.0));
    ros::spinOnce();

    // if(askContinue("Posa"))
    //     robot.goTo(q_spirale, ros::Duration(10.0));
    // ros::spinOnce();

    int count = 0;
    if(askContinue("Iniziare la cattura della point Cloud?")) 
    {
        while(ros::ok()) 
        {
            
            // try {
            //     transform = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); 
            // }
            // catch (tf2::TransformException &ex) {
            //     ROS_WARN("%s",ex.what());
            //     ros::Duration(1.0).sleep();
            // }

            //std::cout << transform << std::endl;

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

            sensor_msgs::PointCloud cloud_temp;
            sensor_msgs::PointCloud2 cloud2;
            //nuvola = bag_read<sensor_msgs::PointCloud2>("/home/workstation2/ws_cross_modal/bags/Prova.bag", "/camera/depth/color/points");
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
    }
    else {}

    return 0;
}