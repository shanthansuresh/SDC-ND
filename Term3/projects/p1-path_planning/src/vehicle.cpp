#include "vehicle.h"

void Vehicle::set_car_frenet_state(double _s_pos, double _s_vel, double _s_acc,
        double _d_pos, double _d_vel, double _d_acc) {

    s_pos = _s_pos;
    s_vel = _s_vel;
    s_acc = _s_acc;
    d_pos = _d_pos;
    d_vel = _d_vel;
    d_acc = _d_acc;
}

/*
 * Returns the Frenet coordinates of the vehicle 
 * for a future time.
 * Assumptions made:
 *  (a) Vehicle travels at constant velocity.
 *  (b) Vehicle stays in the same lane. (TODO: Revisit this.)
 */
vector<double> Vehicle::state_at(double t) const {
    double s_future = s_pos + s_vel*t;

    return {s_future, d_pos};
}

double Vehicle::get_s(void) {
    return s_pos;
}

double Vehicle::get_s_dot(void) {
    return s_vel;
}
