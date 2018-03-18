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

using namespace std;

// for convenience
using json = nlohmann::json;

// System State
struct system_state {

  double target_x = 30.0;
  double initial_spline_points_spacing = 30.0;
  int total_path_points = 50;
  double cycle_period = 0.02; // car starts at slowest lane, most left lane is 0, total number of lanes is 3
  int lane = 1; // car starts at slowest lane, most left lane is 0, total number of lanes is 3
  double lane_size = 4.0; // lane width
  double top_velocity = 50.0; // m/h (Top velocity is 50 m/h)
  double reference_velocity = 49.5; // m/h 
  double next_s = 0.0; // next s point for car position
  double next_d = 0.0; // next d point for car position
  double next_x = 0.0; // next x point for car position
  double next_y = 0.0; // next x point for car position

  // Car will be referenced from started point or from the last point of previous path
  double car_x = 0.0; // reference x car position
  double car_y = 0.0; // reference y car position
  double car_yaw = 0.0; // reference car angle
  double car_s = 0.0; // reference s car position
  double ref_x = 0.0; // reference x car position
  double ref_y = 0.0; // reference y car position
  double ref_yaw = 0.0; // reference car angle

  // Map data
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Previous path
  int prev_size = 0;
  vector<double> previous_path_x;
  vector<double> previous_path_y;

  // Future points to create a spline
  vector<double> future_spline_points_x;
  vector<double> future_spline_points_y;

  // Sensor fusion
  vector<vector<double>> sensor_fusion;  

  // Path spline
  tk::spline path_spline;

  // next values for path
  vector<double> next_x_vals;
  vector<double> next_y_vals;

} _ppss; //Path Planner System State


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

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

// Transform a point to local car coords
vector<double>  trasnform_to_car_coords(double x, double y) {
  vector<double> transformed_point;
  double shift_x = x - _ppss.ref_x;
  double shift_y = y - _ppss.ref_y;

  transformed_point.push_back( shift_x * cos(-_ppss.ref_yaw) - shift_y * sin(-_ppss.ref_yaw) );
  transformed_point.push_back( shift_x * sin(-_ppss.ref_yaw) + shift_y * cos(-_ppss.ref_yaw) );

  return transformed_point;
}

  // Rotate back to world coord system
vector<double>  trasnform_to_world_coords(double x, double y) {
  vector<double> transformed_point;
  double world_x_point = x * cos(_ppss.ref_yaw) - y * sin(_ppss.ref_yaw);
  double world_y_point = x * sin(_ppss.ref_yaw) + y * cos(_ppss.ref_yaw);

  world_x_point += _ppss.ref_x;
  world_y_point += _ppss.ref_y;

  transformed_point.push_back(world_x_point);
  transformed_point.push_back(world_y_point);

  return transformed_point;
}


void initState() {

// Init state for this cycle
  _ppss.future_spline_points_x.clear();
  _ppss.future_spline_points_y.clear();

  _ppss.next_x_vals.clear();
  _ppss.next_y_vals.clear();

  // Use the car as starting ref if the prev path is tiny
  double ref_x_prev = 0.0;
  double ref_y_prev = 0.0;

  if(_ppss.prev_size < 2) {
    _ppss.ref_x = _ppss.car_x;
    _ppss.ref_y = _ppss.car_y;
    ref_x_prev = _ppss.car_x - cos(_ppss.car_yaw);
    ref_y_prev = _ppss.car_y - sin(_ppss.car_yaw);
    _ppss.ref_yaw = _ppss.car_yaw;
  }
  else {
    //Use 2 last points to get a tangent
    _ppss.ref_x = _ppss.previous_path_x[_ppss.prev_size - 1];
    _ppss.ref_y = _ppss.previous_path_y[_ppss.prev_size - 1];

    ref_x_prev = _ppss.previous_path_x[_ppss.prev_size - 2];
    ref_y_prev = _ppss.previous_path_y[_ppss.prev_size - 2];
    _ppss.ref_yaw = atan2(_ppss.ref_y - ref_y_prev, _ppss.ref_x - ref_x_prev);
  }

  vector<double> ref_prev_car_coords = trasnform_to_car_coords(ref_x_prev, ref_y_prev);
  vector<double> ref_car_coords = trasnform_to_car_coords(_ppss.ref_x, _ppss.ref_y);

  _ppss.future_spline_points_x.push_back(ref_prev_car_coords[0]);
  _ppss.future_spline_points_x.push_back(ref_car_coords[0]);

  _ppss.future_spline_points_y.push_back(ref_prev_car_coords[1]);
  _ppss.future_spline_points_y.push_back(ref_car_coords[1]);

  // Create 3 more points ahead of the car spaced by 30m defined in initial_spline_points_spacing
  for(int i = 1; i <= 3; ++i) {
    vector<double> next_point = getXY(_ppss.car_s + i * _ppss.initial_spline_points_spacing,
                                           (2 + _ppss.lane_size * _ppss.lane),
                                           _ppss.map_waypoints_s,
                                           _ppss.map_waypoints_x,
                                           _ppss.map_waypoints_y);

    vector<double> next_point_car_coords = trasnform_to_car_coords(next_point[0], next_point[1]);
    _ppss.future_spline_points_x.push_back( next_point_car_coords[0] );
    _ppss.future_spline_points_y.push_back( next_point_car_coords[1] );
  }

  // Calculate the spline
 _ppss.path_spline.set_points(_ppss.future_spline_points_x, _ppss.future_spline_points_y);

  // Init next vals with the remaining portion of previous path  
  for(int i = 0; i < _ppss.prev_size; i++) {
  _ppss.next_x_vals.push_back(_ppss.previous_path_x[i]);
  _ppss.next_y_vals.push_back(_ppss.previous_path_y[i]);
  }
}

void calculatePath() {
  initState(); //Initialize variables and car ref for current cycle

 double target_x = _ppss.target_x;
 double target_y = _ppss.path_spline(target_x);
 double target_distance = distance(target_x, target_y, 0, 0);
 double x_offset = 0.0;

 // Fill the rest of the path
 for(int i = 1; i <= _ppss.total_path_points - _ppss.prev_size; i++) {
  double steps_count = target_distance / (_ppss.cycle_period * _ppss.reference_velocity / 2.24);
  double next_x_point = x_offset + target_x / steps_count;
  double next_y_point = _ppss.path_spline(next_x_point);

  x_offset = next_x_point;

  // Rotate back to world coord system
  vector<double> world_next_point = trasnform_to_world_coords(next_x_point, next_y_point);

  _ppss.next_x_vals.push_back(world_next_point[0]);
  _ppss.next_y_vals.push_back(world_next_point[1]);
 }

}

int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

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
  	_ppss.map_waypoints_x.push_back(x);
  	_ppss.map_waypoints_y.push_back(y);
  	_ppss.map_waypoints_s.push_back(s);
  	_ppss.map_waypoints_dx.push_back(d_x);
  	_ppss.map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //cout << sensor_fusion << endl;

          	json msgJson;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            _ppss.previous_path_x.clear();
            _ppss.previous_path_y.clear();
            _ppss.prev_size = previous_path_x.size();
            _ppss.sensor_fusion.clear();

            for(int i = 0; i < previous_path_x.size(); ++i) {
              _ppss.previous_path_x.push_back(previous_path_x[i]);
              _ppss.previous_path_y.push_back(previous_path_y[i]);
            }

            for(int i = 0; i < sensor_fusion.size(); i++) {
              vector<double> traffic_car_data;
              for(int j = 0; j < sensor_fusion[i].size(); j++) {
                traffic_car_data.push_back(sensor_fusion[i][j]);
              }
              _ppss.sensor_fusion.push_back(traffic_car_data);
            }

            _ppss.car_x = car_x;
            _ppss.car_y = car_y;
            _ppss.car_yaw = deg2rad(car_yaw);
            _ppss.car_s = car_s;
            calculatePath();

          	msgJson["next_x"] = _ppss.next_x_vals;
          	msgJson["next_y"] = _ppss.next_y_vals;

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
