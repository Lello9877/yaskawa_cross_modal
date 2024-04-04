#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <sun_robot_ros/ClikClient.h>

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

// Metodo goTo in cartesiano, parametri di ingresso:
// posa desiderata, duration, time, bool wait, bool trapez, double trapez lin, double trapez vel

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    geometry_msgs::Pose start, posa, end_effector;

    start.position.x = 0.40;
    start.position.y = 0.10;
    start.position.z = 0.30;
    // una buona posa di partenza, in 15 s
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
    ros::Rate rate(10.0);

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

    if(askContinue("Posizione di partenza")) {
        robot.goTo(start, ros::Duration(15.0));
        ROS_INFO_STREAM("Posa in cartesiano raggiunta");
    }

        if(askContinue("Discesa lungo z")) {
        robot.goTo(posa, ros::Duration(5.0));
        ROS_INFO_STREAM("Posa in cartesiano raggiunta");
    }

    // ESPLORAZIONE

    // double x0, y0, z0;
    // double xf, yf, zf;
    // double divx, divy, deltax, deltay, deltaz;

    // x0 = 0;
    // y0 = -0.25;
    // z0 = 0.25;
    // xf = 0.70;
    // yf = -0.025;
    // divx = 4;
    // divy  = 8;
    // deltax = (xf-x0)/divx;
    // deltay = (yf-y0)/divy;
    // deltaz = -0.03;
    // std::vector<geometry_msgs::Pose> grid;
    // geometry_msgs::Pose temp;
    // geometry_msgs::Quaternion orient;

    // orient.w = cos(M_PI_2/2);
    // orient.x = sin(M_PI_2/2);
    // orient.y = 0;
    // orient.z = 0;
    // temp.orientation = orient;
    
    // for(int i = 0; i < divx; i++)
    //     for(int j = 0; j < divy; j++) {
    //         temp.position.x = x0 + i*deltax;
    //         temp.position.y = y0 + j*deltay;
    //         temp.position.z = z0;
    //         grid.push_back(temp);
    //     }
    
    // for(int i = 0; i < grid.size(); i++)
    //     if(askContinue("Prossima posa")) {
    //         double z = z0;
    //         geometry_msgs::Pose esplor;
    //         esplor = grid[i];
    //         //robot.goTo(grid[i], ros::Duration(5.0));
    //         do
    //         {   
    //             //{condizione di tocco del cavo, altrimenti scendi ancora}
    //             z = z + deltaz;
    //             esplor.position.z = z;
    //             //robot.goTo(esplor, ros::Duration(1.5));
    //             ROS_INFO_STREAM("Posa raggiunta");

    //         } while(ros::ok() && z > 0.02);
    //     }
}