#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_broadcaster");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate loop_rate(10);

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.velocity.resize(4);
    joint_state.effort.resize(4);

    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();

        // Example joint names and states
        joint_state.name[0] = "joint1";
        joint_state.position[0] = 1.0;
        joint_state.velocity[0] = 0.1;
        joint_state.effort[0] = 0.01;

        joint_state.name[1] = "joint2";
        joint_state.position[1] = 1.5;
        joint_state.velocity[1] = 0.2;
        joint_state.effort[1] = 0.02;

        joint_state.name[2] = "joint3";
        joint_state.position[2] = 2.0;
        joint_state.velocity[2] = 0.3;
        joint_state.effort[2] = 0.03;

        joint_state.name[3] = "joint4";
        joint_state.position[3] = 2.5;
        joint_state.velocity[3] = 0.4;
        joint_state.effort[3] = 0.04;

        joint_pub.publish(joint_state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}