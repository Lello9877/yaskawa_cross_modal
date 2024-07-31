#include <ros/ros.h>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include "yaskawa_cross_modal/TactileAction.h"
#include "yaskawa_cross_modal/VisualAction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<yaskawa_cross_modal::TactileAction> ac("tactile_action", true);
    actionlib::SimpleActionClient<yaskawa_cross_modal::VisualAction> ac2("visual_action", true);
    ac.waitForServer();

    int rows = 6, cols = 2;
    double dmin = 0.015;
    double x_max = 0.025, x_min = -0.02, y_max = -0.35, y_min = -0.395, z_start = 0.270;
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

    ac.sendGoalAndWait(goal);

    //ac2.waitForServer();
    yaskawa_cross_modal::VisualGoal goal2;
    goal2.duration = 5.0;
    goal2.q0 = {-1.5594812631607056, -0.32599368691444397, 0.0, -0.13230204582214355, 0.031120063737034798, -1.3393893241882324, -0.022879714146256447};
    //ac2.sendGoalAndWait(goal2);

    return 0;
}