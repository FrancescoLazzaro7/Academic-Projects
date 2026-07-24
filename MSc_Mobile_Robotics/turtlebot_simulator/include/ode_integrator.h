#ifndef ODE_INTEGRATOR_H
#define ODE_INTEGRATOR_H

#include <vector>
#include <boost/numeric/odeint.hpp>

// State format: [x, y, theta, v_actual, omega_actual]
// Dimension increased from 3 to 5 to handle actuator dynamics
using state_type = std::vector<double>;

class OdeIntegrator
{
public:
    // Constructor now takes Ta (actuator time constant)
    explicit OdeIntegrator(double stepSize, double Ta);

    void setInitialState(double x0, double y0, double theta0);
    void setInputs(double v_cmd, double omega_cmd);
    void integrateStep();

    // Updated getter to return actual velocities as well
    void getState(double &x, double &y, double &theta, double &v, double &omega) const;
    void getTime(double &t) const;

private:
    void model(const state_type &s, state_type &dsdt, double t);

    state_type state; // Size 5
    double time = 0.0;
    double dt;
    double Ta;        // Motor time constant

    double v_cmd_target;     // The commanded input
    double omega_cmd_target; // The commanded input

    boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;
};

#endif // ODE_INTEGRATOR_H