#include <stdlib.h>
#include <math.h>
#include "behavior_planner.h"

//#define ENABLE_DEBUG 1

//Convert (m/s) to (mi/hr).
double BehaviorPlanner::convert_speed(double v) {
    return (36.0/16.0)*v;
}

void BehaviorPlanner::check_proximity (vector<vector<double>> sensor_fusion,
    double car_s, double car_d, int prev_points)
{
    double min_dist_ahead = 1000.0;
    this->check_distance_ahead = 1000;
    this->check_speed_ahead = 1000;
    this->too_close = 0;

    for(int i=0; i<sensor_fusion.size(); i++) {   
        //determine if car is directly ahead
        double sensor_d = sensor_fusion[i][6];
        if (fabs(sensor_d - car_d) < 2.0) {   
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed_ahead = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            //check_car_s += (double)prev_points*.02*this->check_speed_ahead;
            check_car_s += (double)prev_points*.02*check_speed_ahead;
            double check_distance_ahead = check_car_s-(car_s+prev_points*.02*this->ego_car_speed);

            //Check for wrap around from max_s to zero
            if (check_distance_ahead > (this->max_s/2.0)) {
                check_distance_ahead -= this->max_s;
            }
            if (check_distance_ahead < (-1.0*this->max_s/2.0)) {
                check_distance_ahead += this->max_s;
            }

            if ((check_distance_ahead > 0) && (check_distance_ahead < min_dist_ahead)) {
                min_dist_ahead = check_distance_ahead;
                this->check_distance_ahead = check_distance_ahead;
                this->check_speed_ahead = check_speed_ahead;
            }

            //if((check_distance_ahead > 0) && (check_distance_ahead<25)) {
            if(min_dist_ahead<25) {
                this->too_close = true;
            }
        }   
    }   

#ifdef ENABLE_DEBUG
    cout << "too_close: " << this->too_close << ", " << this->check_distance_ahead << ", " << (36.0/16.0)*this->check_speed_ahead << endl;
#endif
}

void BehaviorPlanner::adjust_ego_car_speed(void) {
    if(this->too_close) {
#ifdef ENABLE_DEBUG
        cout << "Caution!!! Too close" << endl;
#endif
        this->ego_car_speed = max(this->ego_car_speed -= 0.223, this->min_ego_speed);
        return;
    } 

    //Speedup egocar if falling behind vehicle ahead.
    if ((30 < this->check_distance_ahead) && (this->check_distance_ahead < 50)) {
        if (this->ego_car_speed < this->check_speed_ahead*0.90) { 
#ifdef ENABLE_DEBUG
                cout << "Trying to catch up with car ahead" << endl;
#endif
                this->ego_car_speed = min(this->ego_car_speed += 0.223, this->check_speed_ahead);
        }
    }

    if (this->check_distance_ahead > 50) {
        if (this->ego_car_speed < this->speed_limit*0.99) {
#ifdef ENABLE_DEBUG
                cout << "Trying to catch up with max car speed" << endl;
#endif
                this->ego_car_speed = min(this->ego_car_speed += 0.223, speed_limit);
        }
    }
}

double BehaviorPlanner::get_ego_car_speed(void) {
    return this->ego_car_speed;
}

void BehaviorPlanner::choose_lane(double car_s, double car_d,
    vector<vector<double>> sensor_fusion,
    vector<double> lane_positions, int current_lane, int prev_points) {

    if (this->count >= 0) {
        this->count -= 1;
        return;
    }

    vector<double> lane_traffic_speed;
    vector<double> lane_traffic_speed_behind;
    vector<double> clearance_ahead;
    vector<double> clearance_behind;

    int initial_lane = current_lane;
    this->best_lane = current_lane;

    for (int j=0; j<lane_positions.size(); j++)
    {
        lane_traffic_speed.push_back(1000.0);
        lane_traffic_speed_behind.push_back(-1000.0);
        clearance_ahead.push_back(1000.0);
        clearance_behind.push_back(1000.0);

        for(int i=0; i<sensor_fusion.size(); i++) {
            double sensor_d = sensor_fusion[i][6];
            //if (fabs(sensor_d - lane_positions[j]) < 2.7) {
            if (fabs(sensor_d - lane_positions[j]) < 2.0) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += (double)prev_points*.02*check_speed;
                double check_distance = check_car_s-(car_s+prev_points*.02*this->ego_car_speed);

                //adjust check_distance when wrapping around max_s to zero
                if (check_distance > (this->max_s/2.0)) {
                    check_distance -= this->max_s;
                }
                if (check_distance < (-1.0*this->max_s/2.0)) {
                    check_distance += this->max_s;
                }

                if((check_distance > 0.0) && (fabs(check_distance) < clearance_ahead[j])) {
                    clearance_ahead[j] = check_distance;
                    lane_traffic_speed[j] = check_speed;
                }

                if((check_distance < 0.0) && (fabs(check_distance) < clearance_behind[j])) {
                    clearance_behind[j] = check_distance*(-1.0);
                    lane_traffic_speed_behind[j] = check_speed;
                }
            }
        }
    }

    if (current_lane != LEFT_LANE) {
        if ((clearance_ahead[current_lane-1] > MIN_CLEARANCE_AHEAD) &&
            (clearance_behind[current_lane-1] > MIN_CLEARANCE_BEHIND) &&
            (lane_traffic_speed[current_lane-1] > this->ego_car_speed) &&
            (this->ego_car_speed > lane_traffic_speed_behind[current_lane-1])) {
            this->best_lane = current_lane-1;
        }
    }

    if (current_lane != RIGHT_LANE) {
        if ((clearance_ahead[current_lane+1] > MIN_CLEARANCE_AHEAD) &&
            (clearance_behind[current_lane+1] > MIN_CLEARANCE_BEHIND) &&
            (lane_traffic_speed[current_lane+1] > this->ego_car_speed) &&
            (this->ego_car_speed > lane_traffic_speed_behind[current_lane+1])) {
            this->best_lane = current_lane+1;
        }
    }

    if (initial_lane != this->best_lane) {
        cout << "CHANGING LANES: " << "curr_lane: " << initial_lane << ", Next lane: " << this->best_lane << endl;
        //this->count = 100;
    }

    this->count = 100; //TODO: Should it move inside the previous if condition?

#ifdef ENABLE_DEBUG
cout << "Ego_Car: " << "speed: " << convert_speed(this->ego_car_speed) << endl;
cout << "Left_lane: " << "Ahead: (" << clearance_ahead[0] << ", " << convert_speed(lane_traffic_speed[0]) << ")" << endl;
cout << "           " << "Behind: (" << clearance_behind[0] << ", " << convert_speed(lane_traffic_speed_behind[0]) << ")" << endl;

cout << "Center_lane: " << "Ahead: (" << clearance_ahead[1] << ", " << convert_speed(lane_traffic_speed[1]) << ")" << endl;
cout << "             " << "Behind: (" << clearance_behind[1] << ", " << convert_speed(lane_traffic_speed_behind[1]) << ")" << endl;

cout << "Right_lane: " << "Ahead: (" << clearance_ahead[2] << ", " << convert_speed(lane_traffic_speed[2]) << ")" << endl;
cout << "            " << "Behind: (" << clearance_behind[2] << ", " << convert_speed(lane_traffic_speed_behind[2]) << ")" << endl;
#endif
}

int BehaviorPlanner::get_best_lane(void) {
    return this->best_lane;
}
