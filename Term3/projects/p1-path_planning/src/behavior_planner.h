#ifndef _BEHAVIOR_PLANNER_H_
#define _BEHAVIOR_PLANNER_H_

#include "common.h"
#include "vehicle.h"

using namespace std;

#define GO_STRAIGHT                (0x3<<0)
#define GO_STRAIGHT_FOLLOW_VEHICLE (0x3<<2)
#define TAKE_LEFT_LANE             (0x3<<4)
#define TAKE_RIGHT_LANE            (0x3<<6)

class BehaviorPlanner {
public:
    unsigned int get_feasible_next_states(lane_info olane_info, car_state ocar_state,
    vector<Vehicle> env_veh, double collision_buf_length);
};

#endif //_BEHAVIOR_PLANNER_H_
