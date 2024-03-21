#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState cmd_msg;

void cmd_cb(const sensor_msgs::JointStateConstPtr& msg){
    cmd_msg = *msg;
}

int main(int argc, char* argv[]) {
   
    ros::init(argc,argv,"joint_state");
    ros::NodeHandle nh;
    ros::Publisher pub_joints_cmd = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber sub_joint_cmd = nh.subscribe("/motoman_cmd",1,cmd_cb);
    while(ros::ok()) {
        cmd_msg.header.frame_id = "yaskawa";
        cmd_msg.header.stamp = ros::Time::now();
        pub_joints_cmd.publish(cmd_msg);
        ros::spinOnce();
    }

    return 0;

}