#ifndef _JMT_H_
#define _JMT_H_

#include <vector>
#include <random>
#include <map>

#include "common.h"
#include "vehicle.h"

using namespace std;

typedef struct {
    vector<double> coeff;
    //vector<double> coeff_d;    
} Poly; 

class JMT {
public:
    const double car_width = 1;
    const double car_length = 4;
    const double collision_buf_width = car_width;
    //const double collision_buf_length = 5*car_length;
    const double collision_buf_length = 10*car_length; //TODO: Revisit
    vector<vector<double>> find_best_trajectory(car_state ocar_state, unsigned char next_feasible_states,
    plan_params oplan_params, lane_info olane_info, vector<Vehicle> env_veh);

private:
    int horizon;
    std::map<std::string, double> cost_weights = {
                                            {"traffic_buf_cost", 140.0},
                                            {"efficiency_cost", 110.0},
                                            {"acceleration_s_cost", 10.0},
                                            {"acceleration_d_cost", 10.0},
                                            {"jerk_cost", 10.0},
                                            {"lane_departure_cost", 0.05},
                                            {"traffic_cost", 10.0},
                                            };


    std::default_random_engine rand_generator;
    double std_dev = 0.1;
    int num_pert_samples = 10;

    double distance_over_horizon; //Horizon is 120 timesteps
    double velocity_per_timestep;

    void get_stats_per_horizon(plan_params oplan_params);
    vector<vector<double>> generate_goal_points(car_state ocar_state, unsigned char next_feasible_states,
        plan_params oplan_params, lane_info olane_info, vector<Vehicle> env_veh);
    Poly jmt(vector< double> start, vector <double> end, double T);
    vector<vector<double>> find_least_cost_jmt(car_state ocar_state, plan_params oplan_params,
        vector<vector<double>> goal_points, vector<Vehicle> env_veh);
    double evaluate(Poly poly, double x);
    double calculate_cost(pair<Poly, Poly> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles, vector<vector<double>> &all_costs);
    void perturb_goal(vector<double> goal, vector<vector<double>> &goal_points);

    double logistic(double x);
    double buffer_cost(pair<Poly, Poly> const &traj, vector<Vehicle> const &vehicles);
    double collision_cost(pair<Poly, Poly> const &traj, vector<Vehicle> const &vehicles);
};


#endif //_JMT_H_
