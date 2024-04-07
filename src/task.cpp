#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sun_robot_ros/ClikClient.h>
#include <geometry_msgs/PoseArray.h>
#include "yaskawa_cross_modal/grid.h"

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

void grid_cb(const geometry_msgs::PoseArrayPtr& msg){
    
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    geometry_msgs::Pose start, posa, end_effector;

    // una buona posa, in 15 s
    // posa.position.x = 0.60;
    // posa.position.y = 0.10;
    // posa.position.z = 0.30;

    start.position.x = 0.30;
    start.position.y = -0.30;
    start.position.z = 0.24; 
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

    // ESPLORAZIONE

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

    double deltaz = -0.03;
    geometry_msgs::Pose esplor;

    // for(int i = 0; i < grid.poses.size(); i++) {
    //     if(askContinue("Prossima posa di esplorazione")) {
    //         z = z0;
    //         esplor = grid.poses[i];
    //         esplor.position.z = z;
    //         do
    //         {   
    //             if(askContinue("Esplora")) { robot.goTo(esplor, ros::Duration(5.0)); ROS_INFO_STREAM("Posa raggiunta"); }
    //             //{condizione di tocco del cavo, altrimenti scendi ancora}
    //             z = z + deltaz;
    //             esplor.position.z = z;
    //             std::cout << z << std::endl;
    //         } while(ros::ok() && z > 0.13);
    //     }
    //     else { break; } 
    // }
}