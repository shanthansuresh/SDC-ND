#include <iostream>
#include <vector>

//#include "common.h"
#include "path_planner.h"
#include "jmt.h"
#include "behavior_planner.h"

using namespace std;

int PathPlanner::get_current_lane(double car_d) {
    int cur_lane = 0;
    if ((car_d > 4.0) && (car_d <= 8.0)) {
        cur_lane = 1;
    } else if (car_d > 8.0) {
        cur_lane = 2;
    }

    return cur_lane;
}

vector<int> PathPlanner::find_closest_vehicles_ahead(double ego_s, vector<Vehicle> env_veh) {
    
    vector<int> nearest_veh_lanes(3); //We have 3 lanes on each side of the road.
 
    for (int i=0; i<3; i++) {
        double min_s_dist = 99999;
        int best_idx = -1;

        for (int j=0; j<env_veh.size(); j++) {
            if ((int)((env_veh[j].d_pos)/4) == i) {
                double diff_s = env_veh[j].s_pos - ego_s;
                if ((diff_s>0) && (diff_s<min_s_dist)) {
                    min_s_dist = diff_s;
                    best_idx = j;
                }
            }
        }
        nearest_veh_lanes[i] = best_idx;
    }  

    return nearest_veh_lanes;
}

vector<vector<double>> PathPlanner::plan_path (car_state ocar_state,
    vector<Vehicle> env_veh, plan_params oplan_params) {

    lane_info olane_info;
    JMT jmt;
    BehaviorPlanner behavior_plan;

    /*
     * Get current lane
     */
    olane_info.current_lane = get_current_lane(ocar_state.d_pos);
    cout << "current_lane: " << olane_info.current_lane << endl;

    /*
     * Find the nearest vehicles ahead of the ego vehicle, in each
     * of the lanes.
     */
    olane_info.nearest_veh_lanes = find_closest_vehicles_ahead(ocar_state.s_pos, env_veh);

    /*
     * Behavioral Planning
     */
    unsigned int next_feasible_states; 
    next_feasible_states = behavior_plan.get_feasible_next_states(olane_info,
                             ocar_state, env_veh,
                             jmt.collision_buf_length);
     cout << "Next feasible state: " << next_feasible_states << endl;

    /*
     * Jerk minimizing trajectory 
     */
     vector<vector<double>> new_path;
     new_path = jmt.find_best_trajectory(ocar_state, next_feasible_states, oplan_params, olane_info, env_veh);

    return new_path; 
}
