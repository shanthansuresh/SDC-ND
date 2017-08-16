#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "common.h"
#include "vehicle.h"

using namespace std;

class PathPlanner {
public:
    vector<vector<double>> plan_path (car_state ocar_state,
        vector<Vehicle> env_veh, plan_params oplan_params);

private:
    int get_current_lane(double car_d);
   
    vector<int> find_closest_vehicles_ahead(double ego_s, vector<Vehicle> env_veh);
};

#endif //_PATH_PLANNER_H_
