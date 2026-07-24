#include "robot_simulator_node.h"
#include <cmath>

RobotSimulatorNode::RobotSimulatorNode() : integrator(nullptr) {}

RobotSimulatorNode::~RobotSimulatorNode() {
    if (integrator) {
        delete integrator;
    }
}

void RobotSimulatorNode::prepare()
{
    loadParameters();
    setupCommunication();
    ROS_INFO("RobotSimulator: prepare() completed.");
}

void RobotSimulatorNode::loadParameters()
{
    ros::NodeHandle pnh("~");
    double x0 = 0.0;
    double y0 = 0.0;
    double theta0 = 0.0;

    pnh.param("dt", dt, 0.001);
    pnh.param("Ta", Ta, 0.050); // Load Ta, default 0.050s
    pnh.param("x0", x0, 0.0);
    pnh.param("y0", y0, 0.0);
    pnh.param("theta0", theta0, 0.0);

    if (dt <= 0.0) {
        ROS_WARN("Invalid simulator dt %.6f. Falling back to 0.001 s.", dt);
        dt = 0.001;
    }
    if (Ta <= 0.0) {
        ROS_WARN("Invalid actuator time constant %.6f. Falling back to 0.050 s.", Ta);
        Ta = 0.050;
    }

    // Create new integrator instance with Ta
    integrator = new OdeIntegrator(dt, Ta);
    integrator->setInitialState(x0, y0, theta0);
    
    ROS_INFO("Simulator initialized with dt=%.4f, Ta=%.3f", dt, Ta);
}

void RobotSimulatorNode::setupCommunication()
{
    cmdSubscriber = nh.subscribe("/cmd", 1, &RobotSimulatorNode::cmdCallback, this);
    statePublisher = nh.advertise<std_msgs::Float64MultiArray>("/state", 1);
    clockPublisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void RobotSimulatorNode::cmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (!msg || msg->data.size() < 3) {
        ROS_WARN_THROTTLE(1.0,"RobotSimulator: invalid /cmd message");
        return;
    }
    // data[0]=time, data[1]=v, data[2]=omega
    integrator->setInputs(msg->data[1], msg->data[2]);
}

void RobotSimulatorNode::periodicTask()
{
    // 1. Integrate the model one step forward
    integrator->integrateStep();

    double t, x, y, theta, v, omega;
    integrator->getTime(t);
    integrator->getState(x, y, theta, v, omega);

    // 2. Publish clock
    publishClock(t);

    // 3. Publish state
    publishState(t, x, y, theta, v, omega);
}

void RobotSimulatorNode::publishState(double t, double x, double y, double theta, double v, double omega)
{
    std_msgs::Float64MultiArray msg;
    // Data format: [time, x, y, theta, v_actual, omega_actual]
    // We include v and omega to visualize the step response lag
    msg.data = {t, x, y, theta, v, omega}; 
    statePublisher.publish(msg);
}

void RobotSimulatorNode::publishClock(double t)
{
    rosgraph_msgs::Clock clk;
    clk.clock = ros::Time(t);
    clockPublisher.publish(clk);
}

void RobotSimulatorNode::terminate()
{
    ROS_INFO("RobotSimulator: terminate() called.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_simulator");
    RobotSimulatorNode node;

    node.prepare();

    // Retrieve dt for loop frequency
    double dt_rate;
    {
        ros::NodeHandle pnh("~");
        pnh.param("dt", dt_rate, 0.001);
    }
    if (dt_rate <= 0.0) {
        ROS_WARN("Invalid simulator loop dt %.6f. Falling back to 0.001 s.", dt_rate);
        dt_rate = 0.001;
    }

    // WallRate only throttles execution; simulation time is advanced by the
    // integrator and published on /clock.
    ros::WallRate loop(1.0 / dt_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        node.periodicTask();
        loop.sleep();
    }

    node.terminate();
    return 0;
}
