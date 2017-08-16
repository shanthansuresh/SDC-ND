#ifndef _COMMON_
#define _COMMON_

#include <vector>

using namespace std;

typedef struct {
    double s_pos;
    double s_vel;
    double s_acc;
    double d_pos;
    double d_vel;
    double d_acc;
} car_state;

class EgoVehicle {
public:
    car_state ocar_state;
};

typedef struct {
    double horizon;
    double max_speed;
} plan_params;

typedef struct {
    int current_lane;
    vector<int> nearest_veh_lanes;
} lane_info;
#endif //_COMMON_
