#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include <vector>

using namespace std;

class Vehicle {
public:
    double s_pos;
    double s_vel;
    double s_acc;
    double d_pos;
    double d_vel;
    double d_acc;

    void set_car_frenet_state(double _s_pos, double _s_vel, double _s_acc,
        double _d_pos, double _d_vel, double _d_acc);

    vector<vector<double>> future_states = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
                                             {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

    vector<double> state_at(double t) const;
};

#endif //_PLANNER_H_

