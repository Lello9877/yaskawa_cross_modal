#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Trigger.h>

bool srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

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

    geometry_msgs::PoseArray grid;
    geometry_msgs::Pose initial;

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



    return true;
}

int main(int argc, char* argv[]) {

    ros::init(argc,argv,"grid");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("trigger",srv_cb);
    ros::spin();

}