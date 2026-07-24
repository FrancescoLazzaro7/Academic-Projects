#include "ode_integrator.h"
#include <cmath>
#include <iostream>

OdeIntegrator::OdeIntegrator(double stepSize, double Ta_val) 
    : dt(stepSize), Ta(Ta_val), v_cmd_target(0.0), omega_cmd_target(0.0)
{
    // Initialize state vector with size 5 [x, y, theta, v, w]
    state.resize(5, 0.0);
}

void OdeIntegrator::setInitialState(double x0, double y0, double theta0)
{
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
    state[3] = 0.0; // Initial actual linear velocity
    state[4] = 0.0; // Initial actual angular velocity
}

void OdeIntegrator::setInputs(double v_cmd, double omega_cmd)
{
    v_cmd_target = v_cmd;
    omega_cmd_target = omega_cmd;
}

void OdeIntegrator::integrateStep()
{
    stepper.do_step(
        std::bind(&OdeIntegrator::model, this, 
                  std::placeholders::_1, 
                  std::placeholders::_2, 
                  std::placeholders::_3),
        state,
        time,
        dt
    );
    time += dt;
}

// Extended Unicycle Model Implementation
void OdeIntegrator::model(const state_type &s, state_type &dsdt, double)
{
    // Unpack State
    double theta = s[2];
    double v_curr = s[3];
    double w_curr = s[4];

    // 1. Kinematics based on ACTUAL velocity (not commanded)
    dsdt[0] = v_curr * std::cos(theta); // x_dot
    dsdt[1] = v_curr * std::sin(theta); // y_dot
    dsdt[2] = w_curr;                   // theta_dot

    // 2. Actuator Dynamics (First order lag)
    // v_dot = (v_cmd - v_actual) / Ta
    dsdt[3] = (v_cmd_target - v_curr) / Ta;
    
    // w_dot = (w_cmd - w_actual) / Ta
    dsdt[4] = (omega_cmd_target - w_curr) / Ta;
}

void OdeIntegrator::getState(double &x, double &y, double &theta, double &v, double &omega) const
{
    x = state[0];
    y = state[1];
    theta = state[2];
    v = state[3];
    omega = state[4];
}

void OdeIntegrator::getTime(double &t) const
{
    t = time;
}