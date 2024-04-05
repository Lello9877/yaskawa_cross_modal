#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sun_robot_ros/ClikClient.h>
#include <geometry_msgs/PoseArray.h>

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

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    geometry_msgs::Pose start, posa, end_effector;

    start.position.x = 0.30;
    start.position.y = -0.30;
    start.position.z = 0.24;
    // una buona posa, in 15 s
    // posa.position.x = 0.60;
    // posa.position.y = 0.10;
    // posa.position.z = 0.30;
    start.orientation.w = 0;
    start.orientation.x = 0;
    start.orientation.y = 1;
    start.orientation.z = 0;
    //const std::vector<double> qf = {-1.53, -0.42, 0, -0.52, 0, -0.94, 1.13};
    const std::vector<double> q0 = {0, 0, 0, 0, 0, 0, 0};

    posa.position.x = 0.40;
    posa.position.y = 0.10;
    posa.position.z = 0.05;
    posa.orientation.w = 0;
    posa.orientation.x = 0;
    posa.orientation.y = 1;
    posa.orientation.z = 0;

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

    //     if(askContinue("Discesa lungo z")) {
    //     robot.goTo(posa, ros::Duration(5.0));
    //     ROS_INFO_STREAM("Posa in cartesiano raggiunta");
    // }

    // ESPLORAZIONE

    double x0, y0, z0, z;
    double xf, yf, zf;
    double divx, divy, deltax, deltay, deltaz;

    x0 = 0.30;
    y0 = -0.30;
    z0 = 0.21;
    xf = 0.45;
    yf = 0.30;
    divx = 3;
    divy = 15;
    deltax = (xf-x0)/divx;
    deltay = (yf-y0)/divy;
    deltaz = -0.03;

    //std::vector<geometry_msgs::Pose> grid;
    geometry_msgs::PoseArray grid;
    geometry_msgs::Pose initial, esplor;

    initial.position.z = z0;
    initial.orientation.w = 0;
    initial.orientation.x = 0;
    initial.orientation.y = 1;
    initial.orientation.z = 0;

    // Costruisco la griglia
    for(int i = 0; i < divy; i++) {
        initial.position.y = y0 + i*deltay;
        for(int j = 0; j < divx; j++) {
            initial.position.x = x0 + j*deltax;
            grid.poses.push_back(initial);
        }
    }

    std::cout << grid.poses.size();
    std::cout << grid.poses[44];

    // ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>("/grid",1);
    // while(ros::ok())
    //     pub_poses.publish(grid);

    // for(int i = 0; i < grid.size(); i++)
    //     std::cout << grid[i].position << std::endl;

    for(int i = 0; i < grid.poses.size(); i++) {
        if(askContinue("Prossima posa di esplorazione")) {
            z = z0;
            esplor = grid.poses[i];
            do
            {   
                if(askContinue("Esplora")) { robot.goTo(esplor, ros::Duration(5.0)); ROS_INFO_STREAM("Posa raggiunta"); }
                //{condizione di tocco del cavo, altrimenti scendi ancora}
                z = z + deltaz;
                esplor.position.z = z;
                std::cout << z << std::endl;
            } while(ros::ok() && z > 0.13);
        }
        else { break; } 
    }
}