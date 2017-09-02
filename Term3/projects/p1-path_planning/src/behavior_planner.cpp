#include <iostream>
#include <cmath>

#include "behavior_planner.h"

using namespace std;

unsigned int BehaviorPlanner::get_feasible_next_states(lane_info olane_info,
    car_state ocar_state, vector<Vehicle> env_veh,
    double collision_buf_length) {

    unsigned int next_feasible_states = 0;
    bool go_straight = true;
    bool go_straight_follow_vehicle = false;
    bool take_left_lane = false;
    bool take_right_lane = false;

    vector<int> nearest_veh_lanes = olane_info.nearest_veh_lanes;
    int current_lane = olane_info.current_lane;  
   
   if (nearest_veh_lanes[current_lane] != -1) {
    double closest_veh_s = env_veh[nearest_veh_lanes[current_lane]].get_s();
    // there is some traffic ahead
    if (abs(closest_veh_s - ocar_state.s_pos) < 100) {
      take_left_lane = true;
      take_right_lane = true;
    }   
    
    // there is a vehicle close ahead    
    if (abs(closest_veh_s - ocar_state.s_pos) < collision_buf_length) {
      go_straight = false;
      go_straight_follow_vehicle = true;
      take_left_lane = true;
      take_right_lane = true;
    }   
  }

  if (go_straight == true) {
      next_feasible_states |= GO_STRAIGHT;
  }
  if (go_straight_follow_vehicle == true) {
      next_feasible_states |= GO_STRAIGHT_FOLLOW_VEHICLE;
  }
  if (take_left_lane == true) {
      next_feasible_states |= TAKE_LEFT_LANE;
  }
  if (take_right_lane == true) {
      next_feasible_states |= TAKE_RIGHT_LANE;
  }

  return next_feasible_states;
}
