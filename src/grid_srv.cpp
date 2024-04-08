#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <yaskawa_cross_modal/grid.h>

geometry_msgs::PoseArray grid;
bool call_arrived = false;

bool srv_cb(yaskawa_cross_modal::grid::Request &req, yaskawa_cross_modal::grid::Response &res) {

    geometry_msgs::PoseArray temp_grid;
    geometry_msgs::Pose initial;
    double deltax, deltay;

    deltax = (req.xf-req.x0)/req.divx;
    deltay = (req.yf-req.y0)/req.divy;

    initial.position.z = 0;
    initial.orientation.w = 0;
    initial.orientation.x = 0;
    initial.orientation.y = 1;
    initial.orientation.z = 0;

    // Costruisco la griglia
    for(int i = 0; i < req.divy; i++) {
        initial.position.y = req.y0 + i*deltay;
        for(int j = 0; j < req.divx; j++) {
            initial.position.x = req.x0 + j*deltax;
            temp_grid.poses.push_back(initial);
        }
    }

    call_arrived = true;
    grid = temp_grid;
    grid.header.frame_id = "base_link";
    res.grid = grid;
    if(res.grid.poses.size() == (req.divx*req.divy)) return true;
    else return false;

}

int main(int argc, char* argv[]) {

    ros::init(argc,argv,"grid_srv");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("grid_srv",srv_cb);
    ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>("/grid_poses",1);
    ros::Rate loop_rate(50.0);

    while(ros::ok() && !call_arrived) {
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    grid.header.frame_id = "base_link";

    while(ros::ok()) {
        grid.header.stamp = ros::Time::now();
        pub_poses.publish(grid);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}