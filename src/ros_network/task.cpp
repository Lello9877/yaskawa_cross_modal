#include <ros/ros.h>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include "yaskawa_cross_modal/TactileAction.h"
#include "yaskawa_cross_modal/VisualAction.h"
#include "yaskawa_cross_modal/utility.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<yaskawa_cross_modal::TactileAction> tactile_action("tactile_action", true);
    actionlib::SimpleActionClient<yaskawa_cross_modal::VisualAction> visual_action("visual_action", true);

    visual_action.waitForServer();
    if(askContinue("Avviare la ricostruzione?"))
    {
        std::cout << "Goal inviato" << std::endl;
        yaskawa_cross_modal::VisualGoal goal;
        goal.q0 = {-1.5594812631607056, -0.32599368691444397, 0.0, -0.13230204582214355, 0.031120063737034798, -1.3393893241882324, -0.022879714146256447};
        visual_action.sendGoalAndWait(goal);
        std::cout << "Cattura della point cloud avvenuta" << std::endl;
    }
    else {}

    return 0;
}