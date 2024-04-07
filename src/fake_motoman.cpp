#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char* argv[]) {
   
    ros::init(argc,argv,"fake_motoman");
    ros::NodeHandle nh;
    ros::Publisher pub_motoman_cmd = nh.advertise<sensor_msgs::JointState>("/motoman_cmd",1);
    sensor_msgs::JointState cmd_msg;
    cmd_msg.position.resize(7);
    for (int i = 0; i < 7 ; i++)
        cmd_msg.position[i] = 1.3;
    while(ros::ok()) {
        pub_motoman_cmd.publish(cmd_msg);
        ros::spinOnce();
    }

    return 0;

}