#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

// This node feeds the simulator with a step command
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/cmd", 1);

    // Wait for subscriber (simulator) to connect
    ros::WallRate wait_rate(10.0);
    while(cmd_pub.getNumSubscribers() == 0 && ros::ok()) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    if (!ros::Time::waitForValid(ros::WallDuration(5.0))) {
        ROS_WARN("ROS time is not valid yet. Continuing with the current clock value.");
    }

    double dt = 0.001;
    nh.param("/robot_simulator/dt", dt, 0.001);
    if (dt <= 0.0) {
        ROS_WARN("Invalid test node dt %.6f. Falling back to 0.001 s.", dt);
        dt = 0.001;
    }

    ros::Rate loop_rate(1.0 / dt);

    ROS_INFO("Test Node Started. Sending Step Command...");
    double start_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        double current_time = ros::Time::now().toSec();
        double elapsed = current_time - start_time;

        std_msgs::Float64MultiArray msg;
        
        // Logic:
        // 0 to 1.0s: Stop (v=0)
        // > 1.0s:    Step to v=1.0 m/s
        double v_cmd = (elapsed > 1.0) ? 1.0 : 0.0;
        double w_cmd = 0.0;

        msg.data = {current_time, v_cmd, w_cmd};
        cmd_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
