#include <iostream>

#include "common.h"
#include "jmt.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#if 1
void JMT::perturb_goal(vector<double> goal, vector<vector<double>> &goal_points) {
    std::normal_distribution<double> norm_dist(0.0, std_dev);
    vector<double> pert_goal(6);

    for (int i=0; i<num_pert_samples; i++) {
        double sample = norm_dist(rand_generator);
        pert_goal.at(0) = goal[0] + sample*distance_over_horizon;
        pert_goal.at(1) = goal[1] + sample*velocity_per_timestep;
        pert_goal.at(2) = goal[2];

        sample = norm_dist(rand_generator);
        pert_goal.at(3) = goal[3] + sample;
        pert_goal.at(4) = goal[4];
        pert_goal.at(5) = goal[5];
       
        goal_points.push_back(pert_goal); 
    } 
 
}
#endif

#if 0
double JMT::exceeds_speed_limit_cost(pair<Poly, Poly> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_d(i) + traj.second.eval_d(i) > _hard_max_vel_per_timestep)
      return 1.0;
  }
  return 0.0;
}
#endif


double JMT::calculate_cost(pair<Poly, Poly> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles, vector<vector<double>> &all_costs) {
  Poly s = traj.first;
  Poly d = traj.second;

  double cost = 0.0;

#if 0
  // first situations that immediately make a trajectory infeasible
  double ex_sp_lim_cost = exceeds_speed_limit_cost(traj, goal, vehicles);
  double ex_acc_lim_cost = exceeds_accel_cost(traj, goal, vehicles);
  double ex_jerk_lim_cost = exceeds_jerk_cost(traj, goal, vehicles);
  double col_cost = collision_cost(traj, goal, vehicles);

  double infeasible_costs = ex_sp_lim_cost + ex_acc_lim_cost + ex_jerk_lim_cost + col_cost;
  if (infeasible_costs > 0.0) {
    all_costs.push_back({999999});
    return 999999;
  }

#endif

  double traffic_buf_cost   = buffer_cost(traj, vehicles) * cost_weights["traffic_buf_cost"];
#if 0
  double eff_cost      = efficiency_cost(traj, goal, vehicles) * _cost_weights["eff_cost"];
  double acc_s_cost    = total_accel_s_cost(traj, goal, vehicles) * _cost_weights["acc_s_cost"];
  double acc_d_cost    = total_accel_d_cost(traj, goal, vehicles) * _cost_weights["acc_d_cost"];
  double jerk_cost     = total_jerk_cost(traj, goal, vehicles) * _cost_weights["jerk_cost"];
  double lane_dep_cost = lane_depart_cost(traj, goal, vehicles) * _cost_weights["lane_dep_cost"];
  double traffic_cost  = traffic_ahead_cost(traj, goal, vehicles) * _cost_weights["traffic_cost"];

  vector<double> cost_vec = {tr_buf_cost, eff_cost, acc_s_cost, acc_d_cost, jerk_cost, lane_dep_cost, traffic_cost};
  all_costs.push_back(cost_vec);

#endif
  cost = traffic_buf_cost /*+ eff_cost + acc_s_cost + acc_d_cost + jerk_cost + lane_dep_cost + traffic_cost*/;
  return cost;
}

double JMT::evaluate(Poly poly, double x) {
    double result = 0;

    for (int i = 0; i < poly.coeff.size(); i++) {
//cout << "coeff: " << poly.coeff[i] << ", ";
       result += poly.coeff[i] * pow(x, i); 
    }   
//cout << endl;

    return result;
}

/*
 * Calculate the distance travelled per timestep(20ms),
 * if the car is travelling at 'velocity'(mph)
 */
void JMT::get_stats_per_horizon(plan_params oplan_params) {
   
    //Convert mph to metres/s and account for 20ms time.
    double conversion_factor = (1600.0/3600.0)*0.02;
 
    velocity_per_timestep = conversion_factor * oplan_params.max_speed;
    distance_over_horizon = velocity_per_timestep * oplan_params.horizon;
}

vector<vector<double>> JMT::generate_goal_points(car_state ocar_state, unsigned char next_feasible_states,
    plan_params oplan_params, lane_info olane_info) {
   
    vector<vector<double>> goal_points;
    double goal_s_pos;
    double goal_s_vel;
    double goal_s_acc;
    double goal_d_pos;
    double goal_d_vel;
    double goal_d_acc;
    
    // Go straight
    if (((next_feasible_states>>0) & 0x3) != 0x0) {
        cout << "Going straight" << endl;
        double delta_s = distance_over_horizon;
        goal_s_pos = ocar_state.s_pos + delta_s;
        goal_s_vel = velocity_per_timestep;
        goal_s_acc = 0;
        goal_d_pos = 2 + 4*olane_info.current_lane;
        goal_d_vel = 0;
        goal_d_acc = 0;

        vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};

cout << "goal_vec: " << endl;
for (int kk=0; kk<goal_vec.size(); kk++) {
 cout << "goal_vec: " << goal_vec[kk] << ", ";
}

        vector<vector<double>> goal_points_straight = {goal_vec};
        perturb_goal(goal_vec, goal_points_straight); //TODO: Enable this
        goal_points.reserve(goal_points.size() + goal_points_straight.size());
        goal_points.insert(goal_points.end(),goal_points_straight.begin(),goal_points_straight.end());
    }
    
    // Go straight and follow lead vehicle
    if (((next_feasible_states>>2) & 0x3) != 0x0) {
        cout << "Going straight and following lead vehicle" << endl;
    }
   
    // Switch to left lane
    if (((next_feasible_states>>4) & 0x3) != 0x0) {
        cout << "Switching to left lane" << endl;
    }

    // Switch to right lane
    if (((next_feasible_states>>6) & 0x3) != 0x0) {
        cout << "Switching to right lane" << endl;
    }

//cout << "size of goal_points: " << goal_points.size() << endl;

    return goal_points;
}

#if 0
Poly JMT::jmt(vector< double> start, vector <double> end, double T) {
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
                3*T*T, 4*T*T*T,5*T*T*T*T,
                6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
                end[1]-(start[1]+start[2]*T),
                end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    Poly poly;
    poly.coeff = result;

    return poly;
}
#else
Poly JMT::jmt(vector<double> start, vector<double> goal, double t) {
  double T = double(t);
  double t_2 = pow(T, 2);
  double t_3 = pow(T, 3);
  double t_4 = pow(T, 4);
  double t_5 = pow(T, 5);
  Eigen::Matrix3d A;
  A << t_3,   t_4,    t_5,
       3*t_2, 4*t_3,  5*t_4,
       6*t,   12*t_2, 20*t_3;

  double b_0 = start[0] + start[1] * t + 0.5 * start[2] * t_2;
  double b_1 = start[1] + start[2] * t;
  double b_2 = start[2];
  Eigen::MatrixXd b(3,1);
  b << goal[0] - b_0, goal[1] - b_1, goal[2] - b_2;

  Eigen::MatrixXd c = A.inverse() * b;
  //Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);
  vector<double> coeff = {start[0], start[1], 0.5*start[2], c.data()[0], c.data()[1], c.data()[2]};
  
  Poly poly;
  poly.coeff = coeff;

  return poly;
}

#endif

/*
 * A function that returns a value between 0 and 1 for x in the
 *  range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 *
 * Useful for cost functions.
 */
double JMT::logistic(double x) {
    double y;
   
    y = 2.0 / (1 + exp(-x)) - 1.0;
}

#if 1
/*
 * Penalizes getting close to other vehicles.
 */
double JMT::buffer_cost(pair<Poly, Poly> const &traj, vector<Vehicle> const &vehicles) {
  double cost = 0.0;

  for (int i = 0; i < vehicles.size(); i++) {
    for (int t = 0; t < horizon; t++) {
      double ego_s = evaluate(traj.first, t);
      double ego_d = evaluate(traj.second, t);
      vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}

      double dif_s = traffic_state[0] - ego_s;
      //if (dif_s < -10)
      //  break;
      dif_s = abs(dif_s);
      double dif_d = abs(traffic_state[1] - ego_d);

      // if in the same lane and too close
      if ((dif_s <= collision_buf_length) && (dif_d <= collision_buf_width))
        cost += logistic(1 - (dif_s / collision_buf_length)) / horizon;
    }
  }
  
  return cost;
}
#endif
vector<vector<double>> JMT::find_least_cost_jmt(car_state ocar_state, plan_params oplan_params,
    vector<vector<double>> goal_points, vector<Vehicle> env_veh) {


    vector<pair<Poly, Poly>> traj_coeffs;
    vector<vector<double>> traj_goals; //s, s_dot, s_double_dot, d, d_dot, d_double_dot
    vector<double> traj_costs;

    vector<double> start_s = {ocar_state.s_pos, ocar_state.s_vel, ocar_state.s_acc};
    vector<double> start_d = {ocar_state.d_pos, ocar_state.d_vel, ocar_state.d_acc}; 
 
    for (vector<double> goal : goal_points) {
        vector<double> goal_s = {goal[0], goal[1], goal[2]};
        vector<double> goal_d = {goal[3], goal[4], goal[5]};
   
//cout << "goal[3]: " << goal[3] << endl; 
        if ((goal[3] > 1.0) && (goal[3] < 10.0)) {
            Poly traj_s_poly = jmt(start_s, goal_s, oplan_params.horizon);
            Poly traj_d_poly = jmt(start_d, goal_d, oplan_params.horizon);
            traj_coeffs.push_back(std::make_pair(traj_s_poly, traj_d_poly));
            traj_goals.push_back({goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]});
        }
    }

    /*
     * Compute cost for each trajectory.
     */
    vector<vector<double>> all_costs;
    for (int i = 0; i < traj_coeffs.size(); i++) {
        double cost = calculate_cost(traj_coeffs[i], traj_goals[i], env_veh, all_costs);
        // if appropriate, scale costs for trajectories going to the middle lane
        //if (prefer_mid_lane && (abs(6 - trajectory_coefficients[i].second.eval(_horizon)) < 1.0))
        //  cost *= 0.5;
        traj_costs.push_back(cost);
    }

    /*
     * Find the least cost trajectory.
     */
    double min_cost = traj_costs[0];
    int min_cost_i = 0;
    for (int i = 1; i < traj_coeffs.size(); i++) {
        if (traj_costs[i] < min_cost) {
            min_cost = traj_costs[i];
            min_cost_i = i;
        }
    }

//TODO: Delete this.
//traj_coeffs[min_cost_i].first.coeff = {287.81, 0, 0, 3.22777e-05, -2.45925e-07, 5.26983e-10};
cout << "min_cost_i: " << min_cost_i << endl;

    vector<double> traj_s(oplan_params.horizon);
    vector<double> traj_d(oplan_params.horizon);
    for(int t = 0; t < oplan_params.horizon; t++) {
        traj_s[t] = evaluate(traj_coeffs[min_cost_i].first, t);
        traj_d[t] = evaluate(traj_coeffs[min_cost_i].second, t);
//cout << "(" << traj_s[t] << ", " << traj_d[t] << "),  ";
    }
//cout << endl;

#if 0
cout << "traj_goals: "; 
for (int jj=0; jj<6; jj++) {
 cout << traj_goals[min_cost_i][jj] << endl;
}
cout << endl;
#endif

    vector<vector<double>> new_traj(2);
    new_traj[0] = traj_s;
    new_traj[1] = traj_d;

    return new_traj;
}
 
// Two bits of next_feasible_states correspond to each feasible state
vector<vector<double>> JMT::find_best_trajectory(car_state ocar_state, unsigned char next_feasible_states,
    plan_params oplan_params, lane_info olane_info, vector<Vehicle> env_veh) {
   
    horizon = oplan_params.horizon;

    /*
     * Calculate velocity per timestep(20ms) and distance over horizon.
     */
    get_stats_per_horizon(oplan_params);
 
    /*
     * Generate goal points.
     */
    vector<vector<double>> goal_points;
    goal_points = generate_goal_points(ocar_state, next_feasible_states, oplan_params, olane_info); 
   
    /*
     * Generate jerk minimized trajectory 
     */
    vector<vector<double>> new_path;
    new_path = find_least_cost_jmt(ocar_state, oplan_params, goal_points, env_veh);  
 
    return new_path;
}
