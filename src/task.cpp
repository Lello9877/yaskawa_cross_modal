#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <geometry_msgs/PoseArray.h>
#include "yaskawa_cross_modal/grid.h"
#include "sun_tactile_common/TactileStamped.h"
#include "yaskawa_cross_modal/dynamicArray.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>

bool touched = false;
sun_tactile_common::TactileStamped tensione;

bool askContinue(const std::string &prompt = "")
{
    char ans;
    std::cout << prompt << " - Press y to continue [s to skeep]: ";
    std::cin >> ans;

    if (ans == 'y' || ans == 'Y')
    {
        return true;
    }

    if (ans == 's' || ans == 'S')
    {
        return false;
    }

    throw std::runtime_error("USER STOP!");
}

void tact_cb(const sun_tactile_common::TactileStampedPtr &msg){

    double sum = 0, treshold = 13.99;
    for(int i = 0; i < msg->tactile.data.size(); i++) 
        sum = sum + msg->tactile.data[i];
    
    if(sum <= treshold) { tensione = *msg; touched = false; }
    else touched = true;

    // Somma delle tensioni senza contatto: 13.91, 13.85
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    geometry_msgs::Pose start, posa, end_effector;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl",1);

    // una buona posa, in 15 s
    // posa.position.x = 0.60;
    // posa.position.y = 0.10;
    // posa.position.z = 0.30;

    start.position.x = 0.30;
    start.position.y = -0.30;
    start.position.z = 0.29; 
    start.orientation.w = 0;
    start.orientation.x = 0;
    start.orientation.y = 1;
    start.orientation.z = 0;
    const std::vector<double> q0 = {0, 0, 0, 0, 0, 0, 0};

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(50.0);

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("tool0", "finger", ros::Time(0), ros::Duration(3.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    end_effector.orientation = transformStamped.transform.rotation;
    end_effector.position.x = transformStamped.transform.translation.x;
    end_effector.position.y = transformStamped.transform.translation.y;
    end_effector.position.z = transformStamped.transform.translation.z;
    robot.clik_.set_end_effector(end_effector);

    if(askContinue("Home")) {
        robot.goTo(q0, ros::Duration(5.0));
        ROS_INFO_STREAM("Posa in spazio dei giunti raggiunta");
    }

    if(askContinue("Posizione utile")) {
        robot.goTo(start, ros::Duration(25.0));
        ROS_INFO_STREAM("Posa in cartesiano raggiunta");
    }


    // Eigen::Vector3d p(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z);
    // Eigen::Quaterniond q(transformStamped.transform.rotation.w,transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z);
    // q.normalize();
    // Eigen::Matrix3d R = q.toRotationMatrix();
    // std::cout << p << std::endl;
    // std::cout << R << std::endl;
    // std::cout << R*p;
    
    
    // std::cout << R*p;

    // ESPLORAZIONE

    ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage", 1, tact_cb);
    geometry_msgs::PoseArray grid;
    ros::ServiceClient grid_client = nh.serviceClient<yaskawa_cross_modal::grid>("grid_srv");
    yaskawa_cross_modal::grid srv;
    srv.request.x0 = 0.30;
    srv.request.xf = 0.45;
    srv.request.y0 = -0.30;
    srv.request.yf = 0.30;
    srv.request.divx = 3;
    srv.request.divy = 15;
    if(grid_client.call(srv)) { grid = srv.response.grid; ROS_INFO("Griglia ottenuta"); }
    else { ROS_INFO("Errore nella generazione della griglia"); }
    //std::cout << grid.poses.size();

    double z, z0 = 0.265, deltaz = -0.0070;
    geometry_msgs::Pose esplor;
    int dim = 12;

    double k[dim];
    {
        double h_min = -0.0008, h_max = -0.0006;
        double h = h_max - h_min;
        double v_max[dim] = {1.65, 1.58, 1.44, 1.41, 1.34, 1.13, 3.04, 1.07, 1.28, 1.17, 1.44, 1.03};
        double v_min[dim] = {1, 1.04, 1, 1.06, 1, 0.95, 3.02, 0.90, 0.96, 1, 1.11, 0.86};

    for(int i = 0; i < dim; i++)
        k[i] = h/(v_max[i]-v_min[i]);
    }

    // for(int i = 0; i < 12; i++)
    //     std::cout << k[i] << std::endl;

    std::vector<geometry_msgs::Point32> PCL;

    for(int i = 0; i < grid.poses.size(); i++) {
        //if(askContinue("Prossima posa di esplorazione")) {
            touched = false;
            z = z0;
            esplor = grid.poses[i];
            esplor.position.z = z;
            while(ros::ok() && z > 0.236 && touched == false)
            {   
                /*if(askContinue("Esplora"))*/ { robot.goTo(esplor, ros::Duration(5.0)); ROS_INFO_STREAM("Posa raggiunta"); }
                //{condizione di tocco del cavo, altrimenti scendi ancora}
                z = z + deltaz;
                esplor.position.z = z;
                std::cout << z << std::endl;
                ros::spinOnce();
                if(touched) {
                    std::cout << esplor.position;
                    geometry_msgs::TransformStamped tf_b_to_s;
                    try {
                        tf_b_to_s = tfBuffer.lookupTransform("base_link", "reference_taxel", ros::Time(0), ros::Duration(3.0)); 
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                    }
                    double offset = 0.0035;
                    geometry_msgs::Pose p_si;
                    geometry_msgs::Pose o_b_s;
                    geometry_msgs::Point32 pcl[tensione.tactile.data.size()];
                    
                    o_b_s.position.x = transformStamped.transform.translation.x;
                    o_b_s.position.y = transformStamped.transform.translation.y;
                    o_b_s.position.z = transformStamped.transform.translation.z;
                    o_b_s.orientation = transformStamped.transform.rotation;


                    double refx = o_b_s.position.x;
                    double refy;
                    int count = 0;
                     

                    for(int j = 0; j < tensione.tactile.rows; j++) {
                        refy = o_b_s.position.y; 
                        for(int l = 0; l < tensione.tactile.cols; l++) {
                             pcl[count].x = refx;
                             pcl[count].y = refy;
                             refy = refy + offset;
                             count++;
                        }
                        refx = refx + offset;
                    }

                    for(int i = 0; i < tensione.tactile.data.size(); i++) {
                        pcl[i].z = k[i]*tensione.tactile.data[i];
                        PCL.push_back(pcl[i]);
                        //std::cout << "x: " << pcl[i].x << std::endl << "y: " << pcl[i].y << std::endl << "z: " << pcl[i].z << std::endl << std::endl;
                    }
                }
            }
            int dim = PCL.size();
            std::cout << PCL.size() << std::endl;
            for(int i = 0; i < dim; i++) 
                std::cout << PCL.at(i) << std::endl;

        //}
        //else { break; } 
            sensor_msgs::PointCloud temp;
            temp.header.frame_id = "base_link";
            for(int i = 0; i < PCL.size(); i++) {
                temp.points.push_back(PCL.at(i));
            }
            temp.header.stamp = ros::Time::now();
            pcl_pub.publish(temp);
    }
    return 0;
}