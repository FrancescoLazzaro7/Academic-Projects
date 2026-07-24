#ifndef ROBOT_SIMULATOR_NODE_H
#define ROBOT_SIMULATOR_NODE_H

#include "ros/ros.h"
#include "ode_integrator.h"
#include <std_msgs/Float64MultiArray.h>
#include <rosgraph_msgs/Clock.h>

class RobotSimulatorNode
{
public:
    RobotSimulatorNode();
    ~RobotSimulatorNode();

    void prepare();
    void periodicTask();
    void terminate();

private:
    void loadParameters();
    void setupCommunication();
    void cmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    // Updated to publish v and omega as well for debugging
    void publishState(double t, double x, double y, double theta, double v, double omega);
    void publishClock(double t);

    ros::NodeHandle nh;
    ros::Subscriber cmdSubscriber;
    ros::Publisher statePublisher;
    ros::Publisher clockPublisher;

    OdeIntegrator* integrator;
    double dt;
    double Ta; // Actuator Time Constant
};

#endif // ROBOT_SIMULATOR_NODE_H