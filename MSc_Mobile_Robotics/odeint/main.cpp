#include <vector>
#include <fstream>
#include <iostream>

#include "simulator_odefun.h"

#define DT 0.001
#define M 10.0
#define L 1.0
#define D 4.0
#define X10 0.0
#define X20 0.0
#define NUM_STEP 10000


int main() {
    // Initialize the simulator
    simulator_odefun my_simulator(DT);
    my_simulator.setInitialState(X10, X20);
    my_simulator.setModelParams(M, L, D);

    // Variables to store simulation data
    std::vector<double> t_vect(NUM_STEP+1, 0.0);
    std::vector<double> x1_vect(NUM_STEP+1, 0.0);
    std::vector<double> x2_vect(NUM_STEP+1, 0.0);
    std::vector<double> u_vect(NUM_STEP+1, 0.0);

    // Integrate the model
    for (int k=0; k<=NUM_STEP; k++) {
        // Set the input
        my_simulator.setInputValues(1.0);

        // Integrate
        my_simulator.integrate();

        // Get results
        double x1, x2, t;
        my_simulator.getState(x1, x2);
        my_simulator.getTime(t);

        // Print results every second
        if (std::fabs(std::fmod(t,1.0))<1.0e-3)
            std::cout << "[" << t << "]: " << x1 << ", " << x2 << std::endl;

        // Store variables for plotting
        t_vect.at(k) = t;
        x1_vect.at(k) = x1;
        x2_vect.at(k) = x2;
        u_vect.at(k) = 1.0;

        usleep(1000);
    }

    // Create an output csv file
    std::ofstream resultFile("simulation_results.csv");

    // Write results to file
    resultFile << "t, x1, x2, u\n";
    for(int k=0; k<=NUM_STEP; k++)
    {
        resultFile << t_vect.at(k) << ", " << x1_vect.at(k) << ", " << x2_vect.at(k) << ", " << u_vect.at(k) << "\n";
    }

    // Close the file
    resultFile.close();

    return 0;
}
