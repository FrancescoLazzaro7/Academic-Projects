#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;


class simulator_odefun
{
    public:

        simulator_odefun(double deltaT);

        void setInitialState(double x1, double x2);
        void setModelParams(double m, double l, double d);

        void integrate();
    
        void setInputValues(double u);
    
        void getState(double &x1, double &x2);
        void getTime(double &time);

private:
    // Simulator and integrator variables
    double t, dt;
    double m, l, d;
    double u;

    bool modelParams_set;

    state_type state;
    boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper;

    // ODE function
    void simulator_ode(const state_type &state, state_type &dstate, double t);
};
