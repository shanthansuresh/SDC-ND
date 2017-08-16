#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "common.h"
#include "vehicle.h"
#include "path_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

#if 0
vector<double> spline_fit(double x, double y, double s, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, int num_samples, int flag) {
    

    int prev_wp = -1; 

#if 0
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {   
        prev_wp++;
    }   
#endif
    prev_wp = ClosestWaypoint(x, y, maps_x, maps_y);

    int wp2 = (prev_wp+1)%maps_x.size();
    vector<double> map_wp_xy_upsampled;
  
    //cout << "wp2= " << wp2; 
    vector<double> wp_s_segment;
    vector<double> wp_x_segment;
    vector<double> wp_y_segment;
    
    wp_s_segment.reserve(num_samples);
    wp_x_segment.reserve(num_samples);
    wp_y_segment.reserve(num_samples);
    for (int i=0; i<num_samples; i++) {
        //wp_s_segment[i] = maps_s[wp2+i];
        //wp_x_segment[i] = maps_x[wp2+i];
        //wp_y_segment[i] = maps_y[wp2+i];
cout << "In loop" << endl;
        wp_s_segment.push_back(maps_s[wp2+i]);
        wp_x_segment.push_back(maps_x[wp2+i]);
        wp_y_segment.push_back(maps_y[wp2+i]);
    }

#if 1
    const int samples = int(wp_s_segment[wp_s_segment.size()-1]);
    vector<double> map_wp_s_upsampled;
    map_wp_xy_upsampled.reserve(samples);
    map_wp_s_upsampled.reserve(samples);

 //TODO: Fix the following.
 if (flag == 0) {
    // Fit x coordinates
    tk::spline spline_fit_s_to_x;

cout << "wp_s_segment.size = " << wp_s_segment.size() << endl;
cout << "wp_x_segment.size = " << wp_x_segment.size() << endl;

    spline_fit_s_to_x.set_points(wp_s_segment, wp_x_segment);
    for (int i = 0; i < samples; i++) {
        map_wp_xy_upsampled.push_back(spline_fit_s_to_x(i));
        map_wp_s_upsampled.push_back(i);
    }
} else {
    // Fit y coordinates
    tk::spline spline_fit_s_to_y;

cout << "wp_s_segment.size = " << wp_s_segment.size() << endl;
cout << "wp_y_segment.size = " << wp_y_segment.size() << endl;

    spline_fit_s_to_y.set_points(wp_s_segment, wp_y_segment);
    for (int i = 0; i < samples; i++) {
        map_wp_xy_upsampled.push_back(spline_fit_s_to_y(i));
        //map_wp_s_upsampled.push_back(i);
    }
}
#endif
    return map_wp_xy_upsampled;

}
#endif

#if 0
void sort_coords(vector<double> &v1, vector<double> &v2)
{
    vector<vector<double>> vv;
    int vsize = v1.size();

    for(int i = 0; i < vsize; ++i) {
        vector<double> vt = {v1[i], v2[i]};
        vv.push_back(vt);
    }

    sort(vv.begin(), vv.end(), sort_by_x);

    v1.clear();
    v2.clear();

    for(int i = 0; i < vsize; ++i) {
        if(i > 0 && vv[i][0] == vv[i-1][0]) {
            continue;
        }
        v1.push_back(vv[i][0]);
        v2.push_back(vv[i][1]);
    }
}
#endif

void spline_fit(
    double s,
    vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
    vector<double> maps_dx, vector<double> maps_dy,
    int N_prev, int N_next,
    vector<double> &w_s_local, vector<double> &w_s_world,
    tk::spline &sp_fit_s_x, tk::spline &sp_fit_s_y,
    tk::spline &sp_fit_s_dx, tk::spline &sp_fit_s_dy) {
  
    /*
     * (1) Find where s lies in the waypoint map
     */
    int wp_prev = -1;
    int wp_next = 0;
    while ((maps_s[wp_prev + 1] < s) && (wp_prev < (int)(maps_s.size()-1))) {
        wp_prev++;
    }
  
#if 0
cout << "wp_prev: " << wp_prev << endl;
#endif
 
    /*
     * (2) Choose N_prev index points from behind
     *     and N_next index points in front of s
     */
    vector<double> indices;
    int s_max_idx = maps_s.size();

    //Previous points
    for (int i=N_prev; i>0; i--) {
        //Handle zero crossing os maps_s
        if ((wp_prev-i) < 0) {
            indices.push_back(wp_prev-i+s_max_idx);
        } else {
            indices.push_back(wp_prev-i);
        }
    }
    
    //Next points
    for (int i=0; i<N_next; i++) {
        indices.push_back(wp_prev+i);
    }

    /*
     * (3) Fill the segment of waypoints around s
     */
    double s_max = 6945.554; //Obtained from highway_map.csv
    double s_seg_start = maps_s[indices[0]];
    bool zero_crossed = false;
    vector<double> w_x, w_y, w_dx, w_dy;
    int start_idx = indices[0];
    for (int i=0; i<indices.size(); i++) {
        int cur_idx = indices[i];
//cout << "cur_idx = " << cur_idx << " ";

        w_x.push_back(maps_x[cur_idx]);
        w_y.push_back(maps_y[cur_idx]);
        w_dx.push_back(maps_dx[cur_idx]);
        w_dy.push_back(maps_dy[cur_idx]);

        //if ((i>0) && (cur_idx<indices[i-1])) {
        if ((i>0) && (cur_idx<start_idx)) {
            w_s_local.push_back(s_max-s_seg_start+maps_s[cur_idx]);
        } else {
            w_s_local.push_back(maps_s[cur_idx]-s_seg_start);
        }

        w_s_world.push_back(maps_s[cur_idx]);
    }

#if 0
{
 cout << "s_seg_starte= " << s_seg_start << endl;
 for (int j=0; j<w_s_local.size(); j++) {
     cout << w_s_local[j] << "  ";
 }
 cout << endl;
}
#endif

    /*
     * (4) Fit spline points
     */
    sp_fit_s_x.set_points(w_s_local, w_x);
    sp_fit_s_y.set_points(w_s_local, w_y);
    sp_fit_s_dx.set_points(w_s_local, w_dx);
    sp_fit_s_dy.set_points(w_s_local, w_dy);
}

vector<double> get_xy_points (
    double s, double d,
    tk::spline &sp_fit_s_x, tk::spline &sp_fit_s_y,
    tk::spline &sp_fit_s_dx, tk::spline &sp_fit_s_dy) {
 
    double x_mid = sp_fit_s_x(s);
    double y_mid = sp_fit_s_y(s);
    double dx = sp_fit_s_dx(s);
    double dy = sp_fit_s_dy(s);

    double x = x_mid + d*dx;
    double y = y_mid + d*dy;

    return {x,y};
}

#if 1
double get_local_frenet_coord (double s,
    vector<double> w_s_local, vector<double> w_s_world) {

    int wp = 0;

//cout << __func__ << ", " << s << ", " << w_s_local[0] << ", " << w_s_world[0] << ", ";
 
    // Check if the local frenet segment crosses zero
    if (s < w_s_world[0]) {
        while (w_s_world[wp] != 0.0) {
            wp++;
        }
    }

//cout << wp << ", ";

    //while ((w_s_world[wp] < s) && (w_s_world[wp] != 0.0)) {
    while (w_s_world[wp] < s) {
        wp++;
    }

//cout << wp << ", ";

    double delta_s = s - w_s_world[wp-1];
    double s_local = w_s_local[wp-1] + delta_s;

//cout << s_local << endl;

    return s_local;
}
#endif 

int main() {
  uWS::Hub h;

  PathPlanner PPlanner;
  Vehicle ego_vehicle;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
               &PPlanner, &ego_vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
#if 0
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
#else
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            int prev_path_size = previous_path_x.size();
#endif
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


//Spline fit
#if 0
{
    int num_samples = 10;
    next_x_vals = spline_fit(car_x, car_y, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, num_samples, 0);
    next_y_vals = spline_fit(car_x, car_y, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, num_samples, 1);
}
#endif

#if 1
    vector<double> w_s_local, w_s_world;
    tk::spline sp_fit_s_x, sp_fit_s_y;
    tk::spline sp_fit_s_dx, sp_fit_s_dy;
    int N_prev = 8;
    int N_next = 16;

    spline_fit(car_s,
               map_waypoints_x, map_waypoints_y, map_waypoints_s,
               map_waypoints_dx, map_waypoints_dy,
               N_prev, N_next,
               w_s_local, w_s_world,
               sp_fit_s_x, sp_fit_s_y,
               sp_fit_s_dx, sp_fit_s_dy);
#endif

#if 0
            double dist_inc = 0.1;
            for(int i = 0; i < 50; i++)
            { 
                //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
                double s1 = car_s + i * dist_inc;
                vector<double> xy = getXY(s1, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                /*vector<double> xy = get_xy_points (s1, car_d,
                                                   sp_fit_s_x, sp_fit_s_y,
                                                   sp_fit_s_dx, sp_fit_s_dy);*/

                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
            }
#endif
//cout << "prev path size: " << previous_path_x.size() << endl;

    /*
     * Get local Frenet coordinates for ego car and surrounding vehicles.
     */
    double s_local = get_local_frenet_coord (car_s, w_s_local, w_s_world);
    for (int j=0; j<sensor_fusion.size(); j++) {
        double veh_s = sensor_fusion[j][5];
        sensor_fusion[j][5] = get_local_frenet_coord (veh_s, w_s_local, w_s_world);
    }

#if 1
int cur_lane_i = 0;
            if (car_d > 8) cur_lane_i = 2;
            else if (car_d > 4) cur_lane_i = 1;
cout << "car lane: " << car_d << endl;

#endif
    /*
     * Prepare vehicle objects based on the sensor fusion data
     */
    vector<Vehicle> env_veh(sensor_fusion.size());
    for (int j=0; j<sensor_fusion.size(); j++) {
        double _s_pos = sensor_fusion[j][5];
        double _d_pos = sensor_fusion[j][6];
        double vx = sensor_fusion[j][3];
        double vy = sensor_fusion[j][4];
        double v = (sqrt(pow(vx,2) + pow(vy,2)))*0.02; //Velocity per timestep.
        env_veh[j].set_car_frenet_state(_s_pos, v, 0, _d_pos, 0, 0);
    }  

    /*
     * Plan Path
     */
    car_state ocar_state;
    ocar_state.s_pos = s_local;
    ocar_state.d_pos = car_d;
    int lag = 0; //TODO: Revisit this.
    ocar_state.s_vel = ego_vehicle.future_states[lag][0];
    ocar_state.s_acc = ego_vehicle.future_states[lag][1];
    ocar_state.d_vel = ego_vehicle.future_states[lag][2];
    ocar_state.d_acc = ego_vehicle.future_states[lag][3];
    
    plan_params oplan_params;
    oplan_params.horizon = 120; //TODO: Revisit this
    oplan_params.max_speed = 50; //TODO: Revisit this
    
    vector<vector<double>> new_path;
    new_path = PPlanner.plan_path(ocar_state, env_veh, oplan_params);

    int update_interval = 0; //TODO: Revisit this.
    for (int i = 0; i < 10; i++) {
        double s0 = new_path[0][i + update_interval];
        double s1 = new_path[0][i + update_interval + 1];
        double s2 = new_path[0][i + update_interval + 2];
        double d0 = new_path[1][i + update_interval];
        double d1 = new_path[1][i + update_interval + 1];
        double d2 = new_path[1][i + update_interval + 2];
        double s_v1 = s1 - s0;
        double s_v2 = s2 - s1;
        double s_a = s_v2 - s_v1;
        double d_v1 = d1 - d0;
        double d_v2 = d2 - d1;
        double d_a = d_v2 - d_v1;
        ego_vehicle.future_states[i] = {s_v1, s_a, d_v1, d_a};
    }

    /*
     * Assemble smooth path
     */
#if 1
     bool smooth_path = previous_path_x.size() > 0;

cout << "smooth_path: " << smooth_path << endl;
     double new_x, new_y;
            int smooth_range = 20;
            int reuse_prev_range = 15;
            // start with current car position in x/y
            //vector<double> prev_xy_planned = getXY(new_path[0][0], new_path[1][0], map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
            vector<double> prev_xy_planned;

            // reuse part of previous path, if applicable
            for(int i = 0; i < reuse_prev_range; i++) {
              prev_xy_planned = get_xy_points(new_path[0][i], new_path[1][i], sp_fit_s_x, sp_fit_s_y, sp_fit_s_dx, sp_fit_s_dy);
              if (smooth_path) {
                  // re-use first point of previous path
                  new_x = previous_path_x[i];
                  new_y = previous_path_y[i];
                  next_x_vals.push_back(new_x);
                  next_y_vals.push_back(new_y);
              } else {
                next_x_vals.push_back(prev_xy_planned[0]);
                next_y_vals.push_back(prev_xy_planned[1]);
              }
            }

// assemble rest of the path and smooth, if applicable
            for(int i = reuse_prev_range; i < new_path[0].size(); i++) {
              vector<double> xy_planned = get_xy_points(new_path[0][i], new_path[1][i], sp_fit_s_x, sp_fit_s_y, sp_fit_s_dx, sp_fit_s_dy);
              if (smooth_path) {
                double x_dif_planned =  xy_planned[0] - prev_xy_planned[0];
                double y_dif_planned =  xy_planned[1] - prev_xy_planned[1];
                new_x = new_x + x_dif_planned;
                new_y = new_y + y_dif_planned;
                
                double smooth_scale_fac = (smooth_range - (i - reuse_prev_range)) / smooth_range;
                if (i > smooth_range)
                  smooth_scale_fac = 0.0;
                double smooth_x = (previous_path_x[i] * smooth_scale_fac) + (new_x * (1 - smooth_scale_fac));
                double smooth_y = (previous_path_y[i] * smooth_scale_fac) + (new_y * (1 - smooth_scale_fac));
                
                next_x_vals.push_back(smooth_x);
                next_y_vals.push_back(smooth_y);
                prev_xy_planned = xy_planned;
                
              } else {
                next_x_vals.push_back(xy_planned[0]);
                next_y_vals.push_back(xy_planned[1]);
              }
            }

#endif    

#if 0
cout << "sd: ";
for (int jj=0; jj< next_x_vals.size(); jj++) {
vector<double> sd = getFrenet(next_x_vals[jj], next_y_vals[jj], car_yaw, map_waypoints_x, map_waypoints_y);
cout << sd[0] << ", ";
}
cout << endl;
#endif

#if 0
cout << "new_path.size() " << new_path[0].size();
for (int jj=0; jj< new_path[0].size(); jj++) {
cout << new_path[0][jj] << ", " << new_path[1][jj] << "; ";
}
cout << endl;
#endif


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































