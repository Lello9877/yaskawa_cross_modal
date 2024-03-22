#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState cmd_msg;

void cmd_cb(const sensor_msgs::JointStateConstPtr& msg){
    cmd_msg.position = msg->position;
    cmd_msg.velocity = msg->velocity;
}

int main(int argc, char* argv[]) {
   
    ros::init(argc,argv,"joint_state");
    ros::NodeHandle nh;
    ros::Publisher pub_joints_cmd = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
    ros::Subscriber sub_joint_cmd = nh.subscribe("/motoman_cmd",1,cmd_cb);
    ros::V_string giunti = {"joint_s","joint_l","joint_e","joint_u","joint_r","joint_b","joint_t"};
    int joint_number = 7;
    cmd_msg.position.resize(joint_number);
    cmd_msg.velocity.resize(joint_number);

    for(int i = 0; i < joint_number; i++) {
        cmd_msg.position[i] = 0;
        cmd_msg.velocity[i] = 0;
    }

    cmd_msg.header.frame_id = "yaskawa";
    cmd_msg.name = giunti;

    ros::Rate loop_rate(50.0);
    while(ros::ok()) {
        cmd_msg.header.stamp = ros::Time::now();
        pub_joints_cmd.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}