#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

class ControllerNode
{
public:
    ControllerNode();
    void prepare();
    void run();

private:
    void computeControlInputs(double t, double sample_time, double &v, double &omega);
    
    // Callbacks & Comms
    void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void publishCommand(double t, double v, double omega);
    void publishTrajectory(double t, double x_ref, double y_ref, double x_P, double y_P);
    void loadParameters();

    ros::NodeHandle nh;
    ros::Publisher cmdPublisher;
    ros::Publisher trajPublisher;
    ros::Subscriber stateSubscriber;

    // Trajectory Params
    double epsilon;
    double a, T;
    double dt;
    
    // PI Controller Params
    double Kp, Ti;

    // PI State (Integral Accumulators)
    double integral_x;
    double integral_y;

    // Robot State
    double robot_x, robot_y, robot_theta;

    // Time Synchronization flags
    bool first_msg_received; // Flag to indicate system is ready
    double t0;               // Simulation time when the first packet arrived
    double last_sim_time;
};

#endif // CONTROLLER_NODE_H
