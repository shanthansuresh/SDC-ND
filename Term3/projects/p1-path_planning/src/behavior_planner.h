#ifndef _BEHAVIOR_PLANNER_H_
#define _BEHAVIOR_PLANNER_H_

#include <iostream>
#include <vector>

using namespace std;

#define MIN_CLEARANCE_AHEAD 18
#define MIN_CLEARANCE_BEHIND 12

class BehaviorPlanner {
public:
    BehaviorPlanner() {};
    BehaviorPlanner(double max_speed) {
        speed_limit = max_speed;
    };

    typedef enum {
        LEFT_LANE = 0,
        CENTER_LANE = 1,
        RIGHT_LANE = 2
    } LANE;

    int best_lane = 1;
    double speed_limit = 22.2;
    bool too_close = false;
    double check_distance_ahead;
    double check_speed_ahead;
    double ego_car_speed = 0.0;

    void check_proximity (vector<vector<double>> sensor_fusion,
        double car_s, double car_d, int prev_points);

    void choose_lane(double car_s, double car_d, vector<vector<double>> sensor_fusion,
        vector<double> lane_positions, int current_lane, int prev_points);

    void adjust_ego_car_speed(void);
    double get_ego_car_speed(void);

    int get_best_lane(void);

    int count = -1;

private:
    double max_s = 6945.554; // The max s value before wrapping around the track back to 0
    double min_ego_speed = 6.66; //6.66m/s is equivalent to 15mi/hr.
    double convert_speed(double v);
};
#endif //_BEHAVIOR_PLANNER_H_
