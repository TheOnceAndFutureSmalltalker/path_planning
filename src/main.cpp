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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// useful constants, some duplicated within onMessage() function
const double LANE_WIDTH = 4;      // each lane is 4 meters
const double DELTA_T = 0.02;      // one fiftieth of a second, rate of simulation updates
const double AHEAD_BUFFER = 10;   // car ahead of me in target lane cannot be closer than this
const double BEHIND_BUFFER = 20;  // car behind me in target lane cannot be closer than this
const double MS_TO_MPH = 2.23;    // multiply by this to convert m/s to mph
const double MAX_DELTA_VEL = 0.446;   // this is change in velocity over 0.02 s for max acceleration of 10 m/s^s
const double MAX_VELOCITY = 49.25; // target max velocity just under speed limit
const double NUM_PTS_PER_INTERVAL = 3.0;  // min number of 0.02 spaced points usually consumed per request


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

// calculates distance between me and car directly in front of me, returns distance and car's speed in mph
vector<double> carInFront(int lane, double car_s, int prev_size, const vector<vector<double>> &sensor_fusion)
{
  double min_distance_s = 10000;
  double speed = 10000;
  for(int i=0;i<sensor_fusion.size();i++)
  {
    float d = sensor_fusion[i][6];
    if(d >= LANE_WIDTH*lane && d < LANE_WIDTH*(lane+1))  // is car in the lane of interest
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += (double)prev_size*DELTA_T*check_speed;
      if(check_car_s > car_s)
      {
       double distance_s = check_car_s - car_s;
        if(distance_s < min_distance_s)
        {
          min_distance_s = distance_s;
          speed = check_speed * MS_TO_MPH;
        }
      }
    }
  }
  return {min_distance_s, speed};
}

// returns score of ability to change lanes
double scoreLaneChange(int target_lane, int cur_lane, double car_s, double car_speed, int prev_size, const vector<vector<double>> &sensor_fusion)
{
  vector<double> target_lane_front_car = {10000,10000};
  vector<double> target_lane_rear_car = {10000,10000};
  vector<double> cur_lane_front_car = {10000,10000};

  for(int i=0;i<sensor_fusion.size();i++)
  {
    float d = sensor_fusion[i][6];
    if(d >= LANE_WIDTH*cur_lane && d < LANE_WIDTH*(cur_lane+1))  // is car in the current lane
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += (double)prev_size*DELTA_T*check_speed;
      if(check_car_s > car_s) // is car in front of me
      {
        if(check_car_s-car_s < cur_lane_front_car[0]) // car is closer than previous car
        {
          cur_lane_front_car[0] = check_car_s-car_s;
          cur_lane_front_car[1] = check_speed * MS_TO_MPH;
        }
      }
    }
    if(d >= LANE_WIDTH*target_lane && d < LANE_WIDTH*(target_lane+1))  // is car in the target_lane
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += (double)prev_size*DELTA_T*check_speed;
      if(check_car_s > car_s) // is car in front of me
      {
        if(check_car_s-car_s < target_lane_front_car[0]) // car is closer than previous car
        {
          target_lane_front_car[0] = check_car_s-car_s;
          target_lane_front_car[1] = check_speed * MS_TO_MPH;
        }
      }
      if(check_car_s < car_s) // is car behind me
      {
        if(car_s-check_car_s < target_lane_rear_car[0]) // car is closer than previous car
        {
          target_lane_rear_car[0] = car_s-check_car_s;
          target_lane_rear_car[1] = check_speed * MS_TO_MPH;
        }
      }
    }
  }

  if(target_lane_front_car[0] < 20)
  { // target lane front car must be at least this far ahead of me
    cout << "front car not far enough ahead of me" << endl;
    return -1;
  }
  if(target_lane_rear_car[0] < 10)
  { // target lane rear car must be at least this far behind me
    cout << "rear car not far enough behind me" << endl;
    return -1;
  }
  if(target_lane_rear_car[1] > car_speed + 1)
  { // if rear car going just a little faster than me
    // calculate how long it would take for him to catch me at my current speed
    double close_time = target_lane_rear_car[0] / ((target_lane_rear_car[1] - car_speed) * 0.447);
    if(close_time < 3)
    {
      cout << "rear car close time is " << close_time << endl;
      return -1;
    }
  }

  // calculate a score based on speed and distance
  double score = 0.2 * (target_lane_front_car[0] - 10) + 1.0 * (target_lane_front_car[1] - cur_lane_front_car[1] - 3);
  cout << "score:  " << score << endl;
  return score;
}

// updates ref_vel to keep my car within target_distance of car in front
void followThatCar(double &ref_vel, double car_speed, double car_in_front_speed, double car_in_front_distance, double target_distance)
{
  if(car_in_front_distance < target_distance)
  {
    // too close, get to 5 mph slower than car in front
    if(car_speed + 5 > car_in_front_speed)
    {
      ref_vel -= MAX_DELTA_VEL * NUM_PTS_PER_INTERVAL;
    }
    else
    {
      ref_vel += MAX_DELTA_VEL * NUM_PTS_PER_INTERVAL;
    }
  }
  else if(car_in_front_distance < target_distance + 10)
  {
    // approaching safe distance, try to be about 5 mph faster than car in front
    if(car_speed - 5 > car_in_front_speed)
    {
      ref_vel -= MAX_DELTA_VEL * NUM_PTS_PER_INTERVAL;
    }
    if(car_speed - 5 < car_in_front_speed)
    {
      ref_vel += MAX_DELTA_VEL * NUM_PTS_PER_INTERVAL;
    }
  }
  else
  {
    if(car_speed < 0.5)
    {
      ref_vel += MAX_DELTA_VEL; // from dead stop, assume pure 0.02 s increments
    }
    else
    {
      ref_vel += MAX_DELTA_VEL * NUM_PTS_PER_INTERVAL;
    }
  }
  if(ref_vel > MAX_VELOCITY)
  {
    ref_vel = MAX_VELOCITY;
  }
}

int main() {
  uWS::Hub h;

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




  // these variables are subject to change and must be passed in to the onMessage function
  // current lane of the my car, starts in center lane
  int lane = 1;
  // velocity used to calculate path
  double ref_vel = 0.0;  //mph
  // current state of car, KL, LCL, LCR, PLC
  string state = "KL";

  cout << state << endl;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // these are constants used throughout
          const double DELTA_T = 0.02;      // one fiftieth of a second, rate of simulation updates
          const double LEFT_LANE = 0;
          const double CENTER_LANE = 1;
          const double RIGHT_LANE = 2;
          const double SAFE_DISTANCE = 30;  // meters buffer between me and car in front of me
          const double LANE_WIDTH = 4;      // each lane is 4 meters
          const string KEEP_LANE = "KL";     // Keep Lane
          const string LANE_CHANGE_LEFT = "LCL";   // Lane Change Left
          const string LANE_CHANGE_RIGHT = "LCR";   // Lane Change Right
          const string PREPARE_LANE_CHANGE = "PLC";   // Prepare for Lane Change
          const double MAX_DELTA_VEL = 0.446;   // this is change in velocity over 0.02 s for max acceleration of 10 m/s^s
          const double MAX_VELOCITY = 49.5; // target max velocity just under speed limit
          const double MS_TO_MPH = 2.23;    // how long it takes in seconds to travel 1 meter going 1 MPH
          const int POINTS_IN_PATH = 50;
          const double ANCHOR_POINT_DIST = 30.0;
          const double NUM_PTS_PER_INTERVAL = 3.0;  // min number of 0.02 spaced points consumed per request
          const double MIN_LC_SCORE = 5.0;


          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"]; // in degrees
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();
          	if(prev_size>0)
            {
              car_s = end_path_s;
            }

            // calculate distance and speed of car in front
            vector<double> car_in_front = carInFront(lane, car_s, prev_size, sensor_fusion);
            double car_in_front_distance = car_in_front[0];
            double car_in_front_speed = car_in_front[1];



 /****************************************************************/
 /**     S T A T E   T R A N S I T I O N                        **/
 /****************************************************************/
            string next_state = state;
            if(state == KEEP_LANE)
            {
              // if car in front is moving too slow
              if(car_in_front_speed + 5 < MAX_VELOCITY)
              {
                // and I am too close to car in front of me
                if(car_in_front_distance < SAFE_DISTANCE + 3)
                {
                  // then transition to prepare for lane change
                  next_state = PREPARE_LANE_CHANGE;
                }
              }
            }
            if(state == LANE_CHANGE_RIGHT || state == LANE_CHANGE_LEFT)
            {
              // go back to KL when d is close to center of target lane
              double lane_d = lane * LANE_WIDTH + LANE_WIDTH/2;
              if(abs(car_d - lane_d) < 0.5)
              {
                next_state = KEEP_LANE;
              }
            }
            if(state == PREPARE_LANE_CHANGE)
            {
              if(lane == LEFT_LANE)
              {
                // if I am in the left lane, can I change into the middle lane
                double scoreLCR = scoreLaneChange(CENTER_LANE, LEFT_LANE, car_s, car_speed, prev_size, sensor_fusion);
                if(scoreLCR > MIN_LC_SCORE)
                {
                  next_state = LANE_CHANGE_RIGHT;
                }
              }
              if(lane == RIGHT_LANE)
              {
                // if I am in the right lane, can I change into the center lane
                double scoreLCL = scoreLaneChange(CENTER_LANE, RIGHT_LANE, car_s, car_speed, prev_size, sensor_fusion);
                if(scoreLCL > MIN_LC_SCORE)
                {
                  next_state = LANE_CHANGE_LEFT;
                }
              }
              if(lane == CENTER_LANE)
              {
                // if I am in the center lane, can I change into the left lane
                double scoreLCL = scoreLaneChange(LEFT_LANE, CENTER_LANE, car_s, car_speed, prev_size, sensor_fusion);
                double scoreLCR = scoreLaneChange(RIGHT_LANE, CENTER_LANE, car_s, car_speed, prev_size, sensor_fusion);
                if(scoreLCL > MIN_LC_SCORE || scoreLCR > MIN_LC_SCORE)
                {
                  if(scoreLCL > scoreLCR)
                  {
                    next_state = LANE_CHANGE_LEFT;
                  }
                  else
                  {
                    next_state = LANE_CHANGE_RIGHT;
                  }
                }
              }
            }
            if(next_state != state)
            {
              cout << next_state << endl;
            }



/**********************************************************************/
/**     E X E C U T E   S T A T E   A C T I O N                      **/
/**********************************************************************/
            if(next_state == KEEP_LANE)
            {
              followThatCar(ref_vel, car_speed, car_in_front_speed, car_in_front_distance, SAFE_DISTANCE);
            }
            if(next_state == LANE_CHANGE_LEFT)
            {
              // if transitioning from PCL, set new lane for trajectory calculation
              if(state == PREPARE_LANE_CHANGE)
              {
                lane = lane - 1;
              }
              // start speeding up
              if(ref_vel < MAX_VELOCITY)
              {
                ref_vel += MAX_DELTA_VEL;
              }
              if(ref_vel > MAX_VELOCITY)
              {
                ref_vel = MAX_VELOCITY;
              }
            }
            if(next_state == LANE_CHANGE_RIGHT)
            {
              // if transitioning from PLC, set new lane for trajectory calculation
              if(state == PREPARE_LANE_CHANGE)
              {
                lane = lane + 1;
              }
              // start speeding up
              if(ref_vel < MAX_VELOCITY)
              {
                ref_vel += MAX_DELTA_VEL;
              }
              if(ref_vel > MAX_VELOCITY)
              {
                ref_vel = MAX_VELOCITY;
              }
            }
            if(next_state == PREPARE_LANE_CHANGE)
            {
              // just maintain buffer distance with car in front
              followThatCar(ref_vel, car_speed, car_in_front_speed, car_in_front_distance, SAFE_DISTANCE);
            }

            // update state for next update from simulator
            state = next_state;



/**********************************************************************/
/**     C A L C U L A T E   T R A J E C T O R Y                      **/
/**********************************************************************/
          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	vector<double> ptsx;
          	vector<double> ptsy;

          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	if(prev_size<2)
          	{
          	  double prev_car_x = car_x - cos(car_yaw);
          	  double prev_car_y = car_y - sin(car_yaw);

          	  ptsx.push_back(prev_car_x);
          	  ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
          	  ptsy.push_back(car_y);
          	}
          	else
            {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          	}

            double lane_center_d = LANE_WIDTH/2+LANE_WIDTH*lane;
          	vector<double> next_wp0 = getXY(car_s+ANCHOR_POINT_DIST, lane_center_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+2*ANCHOR_POINT_DIST, lane_center_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+3*ANCHOR_POINT_DIST, lane_center_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	for(int i=0;i<ptsx.size();i++)
            {
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            tk::spline s;

            s.set_points(ptsx,ptsy);

            // start with previous path points
            for(int i=0;i<previous_path_x.size();i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = ANCHOR_POINT_DIST;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            double x_add_on = 0;

            for(int i=1;i<=POINTS_IN_PATH-previous_path_x.size();i++)
            {
              double N = target_dist/(DELTA_T*ref_vel/MS_TO_MPH);
              double x_point = x_add_on+target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

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
