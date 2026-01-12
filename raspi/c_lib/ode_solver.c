
#include "ode_solver.h"

double solve_ode(double t0, double t1, double dt) {
    double y = 1.0; // initial condition
    for (double t = t0; t < t1; t += dt) {
        y += dt * (-y); // dy/dt = -y
    }
    return y;
}
