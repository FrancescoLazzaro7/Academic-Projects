#include "controller_node.h"
#include <cmath>

ControllerNode::ControllerNode() 
    : robot_x(0), robot_y(0), robot_theta(0),
      integral_x(0.0), integral_y(0.0),
      first_msg_received(false), t0(0.0), last_sim_time(0.0) {}

void ControllerNode::prepare()
{
    loadParameters();
    
    // Setup Communication
    cmdPublisher = nh.advertise<std_msgs::Float64MultiArray>("/cmd", 1);
    trajPublisher = nh.advertise<std_msgs::Float64MultiArray>("/trajectory", 1);
    stateSubscriber = nh.subscribe("/state", 1, &ControllerNode::stateCallback, this);
    
    ROS_INFO("Controller Prepared.");
    ROS_INFO("Parameters: dt=%f, Kp=%.2f, Ti=%.2f", dt, Kp, Ti);
}

void ControllerNode::loadParameters()
{
    ros::NodeHandle pnh("~");
    
    pnh.param("epsilon", epsilon, 0.2);
    pnh.param("a", a, 10.0); 
    pnh.param("T", T, 5.0);  
    pnh.param("dt", dt, 0.001);
    
    // PI Gains
    pnh.param("Kp", Kp, 4.0); // Optimized Gain
    pnh.param("Ti", Ti, 0.5); // Optimized Time Constant

    if (dt <= 0.0) {
        ROS_WARN("Invalid controller dt %.6f. Falling back to 0.001 s.", dt);
        dt = 0.001;
    }
    if (epsilon <= 0.0) {
        ROS_WARN("Invalid epsilon %.6f. Falling back to 0.2 m.", epsilon);
        epsilon = 0.2;
    }
    if (T <= 0.0) {
        ROS_WARN("Invalid trajectory period %.6f. Falling back to 5.0 s.", T);
        T = 5.0;
    }
    if (Kp < 0.0) {
        ROS_WARN("Invalid negative Kp %.6f. Falling back to 4.0.", Kp);
        Kp = 4.0;
    }
    if (Ti <= 0.0) {
        ROS_WARN("Invalid controller Ti %.6f. Falling back to 0.5 s.", Ti);
        Ti = 0.5;
    }
}

void ControllerNode::stateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 4) return;

    double msg_time = msg->data[0];
    
    // --- ROBUST RESET LOGIC ---
    // 1. Detect Time Jump (Sim Restart): If current msg time is LESS than the last one.
    // 2. Detect Buffer Glitch: If we just started, but grabbed a 'future' packet by mistake.
    if (first_msg_received) {
        if (msg_time < last_sim_time) {
            ROS_WARN("Time Jump Detected! (%.2f -> %.2f). Resetting Controller.", last_sim_time, msg_time);
            first_msg_received = false; // Force re-latch
        }
    }

    robot_x = msg->data[1];
    robot_y = msg->data[2];
    robot_theta = msg->data[3];

    // --- LATCH SYNCHRONIZATION ---
    if (!first_msg_received) {
        t0 = msg_time;
        first_msg_received = true;

        integral_x = 0.0;
        integral_y = 0.0;
        last_sim_time = msg_time;

        ROS_INFO("Synchronization Locked. T0 = %.4f", t0);

        double v_cmd = 0.0;
        double omega_cmd = 0.0;
        computeControlInputs(0.0, 0.0, v_cmd, omega_cmd);
        publishCommand(msg_time, v_cmd, omega_cmd);
        return;
    }

    const double sample_time = msg_time - last_sim_time;
    if (sample_time <= 0.0) {
        ROS_WARN_THROTTLE(1.0, "Ignoring non-increasing simulator timestamp.");
        return;
    }
    if (sample_time > 5.0 * dt) {
        ROS_WARN_THROTTLE(1.0, "Large controller sample time: %.6f s", sample_time);
    }

    last_sim_time = msg_time;

    double v_cmd = 0.0;
    double omega_cmd = 0.0;
    computeControlInputs(msg_time - t0, sample_time, v_cmd, omega_cmd);
    publishCommand(msg_time, v_cmd, omega_cmd);
}

void ControllerNode::run()
{
    ros::spin();
}

void ControllerNode::computeControlInputs(double t, double sample_time, double &v, double &omega)
{
    // 1. Reference Generation (8-Shape)
    double w = 2 * M_PI / T;
    double tau = w * t;

    double x_ref = a * sin(tau);
    double y_ref = a * sin(tau) * cos(tau);

    // Feedforward Velocities
    double dx_ref = a * w * cos(tau);
    double dy_ref = a * w * cos(2 * tau); 

    // 2. Feedback Linearization
    double P_x = robot_x + epsilon * cos(robot_theta);
    double P_y = robot_y + epsilon * sin(robot_theta);

    // 3. Error Calculation
    double e_x = x_ref - P_x;
    double e_y = y_ref - P_y;

    // 4. PI Control
    integral_x += e_x * sample_time;
    integral_y += e_y * sample_time;

    double u_x = dx_ref + Kp * e_x + (Kp / Ti) * integral_x;
    double u_y = dy_ref + Kp * e_y + (Kp / Ti) * integral_y;

    // 5. Inverse Transformation
    v = u_x * cos(robot_theta) + u_y * sin(robot_theta);
    omega = (u_y * cos(robot_theta) - u_x * sin(robot_theta)) / epsilon;
    
    // Publish Reference for Plotting
    publishTrajectory(t, x_ref, y_ref, P_x, P_y);
}

void ControllerNode::publishCommand(double t, double v, double omega)
{
    std_msgs::Float64MultiArray msg;
    msg.data = {t, v, omega};
    cmdPublisher.publish(msg);
}

void ControllerNode::publishTrajectory(double t, double xr, double yr, double xp, double yp)
{
    std_msgs::Float64MultiArray msg;
    msg.data = {t, xr, yr, xp, yp};
    trajPublisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ControllerNode node;
    node.prepare();
    node.run();
    return 0;
}
