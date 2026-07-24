#include "simulator_odefun.h"


simulator_odefun::simulator_odefun(double deltaT)
{
    // Initialize time and dt
    t = 0.0;
    dt = deltaT;

    // Initialize flags
    modelParams_set = false;

    // Initial state values
    state.resize(2);
    state[0] = 0.0;
    state[1] = 0.0;
}

void simulator_odefun::setInitialState(double x1, double x2)
{
    // Initial state values
    state[0] = x1;
    state[1] = x2;
}

void simulator_odefun::setModelParams(double m, double l, double d)
{
    // Initialize model parameters
    this->m = m;
    this->l = l;
    this->d = d;

    modelParams_set = true;
}

void simulator_odefun::setInputValues(double u)
{
    this->u = u;
}

void simulator_odefun::integrate()
{
    // Check model parameters are set
    if (!modelParams_set) {
        throw std::invalid_argument( "Model parameters not set!" );
    }

    // Integrate for one step ahead
    stepper.do_step(std::bind(&simulator_odefun::simulator_ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), state, t, dt);

    // Update time
    t += dt;
}

void simulator_odefun::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double x1 = state[0];
    const double x2 = state[1];

    // Model equations of a nonlinear pendulum with friction
    dstate[0] = x2;
    dstate[1] = -9.81/l*std::sin(x1)-d/(m*std::pow(l,2.0))*x2+u/(m*std::pow(l,2.0));
}

void simulator_odefun::getState(double &x1, double &x2)
{
    x1 = state[0];
    x2 = state[1];
}

void simulator_odefun::getTime(double &time)
{
    time = t;
}