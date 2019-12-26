#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#define LEFT_LANE 0
#define CENTER_LANE 1
#define RIGHT_LANE 2

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int car_lane = 1;
  double ref_vel = 0.0;

  const double MAX_SPEED = 49.5;

  h.onMessage([&ref_vel, &car_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &MAX_SPEED]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int previous_path_size = previous_path_x.size();

          // For more correct calculation of the next path we want
          // to base those calculation of the car's positioning
          // on the last point of our previous path
          // (in case we didn't already consume it).
          if (previous_path_size > 0) {
            car_s = end_path_s;
          }

          // Check other cars through the sensor fusion data
          // to determine if there's a car nearby.
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            int other_car_lane = -1;

            // Determine car lane
            if (d > 0 && d < 4) {
              other_car_lane = LEFT_LANE;
            } else if (d > 4 && d < 8) {
              other_car_lane = CENTER_LANE;
            } else if (d > 8 && d < 12) {
              other_car_lane = RIGHT_LANE;
            }
            if (other_car_lane < 0) {
              continue;
            }

            // Determine car speed from vx, vy
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double other_car_speed = sqrt(vx*vx + vy*vy);
            double other_car_s = sensor_fusion[i][5];

            // Estimate the car's s position after consuming the previous
            // path to match with our own car.
            other_car_s += ((double)previous_path_size * 0.02 * other_car_speed);

            // We check if there are any nearby cars within our path's distance
            if (other_car_lane == car_lane) {
              car_ahead |= other_car_s > car_s && other_car_s - car_s < 35;
            } else if (other_car_lane - car_lane == -1) {
              car_left |= car_s - 35 < other_car_s && car_s + 35 > other_car_s;
            } else if (other_car_lane - car_lane == 1) {
              car_right |= car_s - 35 < other_car_s && car_s + 35 > other_car_s;
            }
          }

          const double MAX_ACC_ENC = .224;

          // Determine if we should increase or decrease the speed and whether
          // or not we should switch lanes.
          double car_acceleration = 0;
          if (car_ahead) {
            // Change lane to get over the car ahead of us
            if (!car_left && car_lane != LEFT_LANE) {
              car_lane--;
            } else if (!car_right && car_lane != RIGHT_LANE){
              car_lane++;
            } else {
              car_acceleration -= MAX_ACC_ENC;
            }
          } else {
            if (car_lane != 1) {
              if ((car_lane == LEFT_LANE && !car_right) || (car_lane == RIGHT_LANE && !car_left))
                car_lane = 1;
            }

            if (ref_vel < MAX_SPEED) {
              car_acceleration += MAX_ACC_ENC;
            }
          }

          // We want to create a spline to model the trajectory of the next path
          // using a couple of few last points in the previous path and few
          // future points based on the coordinates and directionality of our
          // car to be able to get values of interploated points.
          vector<double> spline_x_vals;
          vector<double> spline_y_vals;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (previous_path_size < 2) {
              double previous_car_x = ref_x - cos(car_yaw);
              double previous_car_y = ref_y - sin(car_yaw);

              spline_x_vals.push_back(previous_car_x);
              spline_y_vals.push_back(previous_car_y);

              spline_x_vals.push_back(ref_x);
              spline_y_vals.push_back(ref_y);
          } else {
              ref_x = previous_path_x[previous_path_size-1];
              ref_y = previous_path_y[previous_path_size-1];

              double previous_ref_x = previous_path_x[previous_path_size-2];
              double previous_ref_y = previous_path_y[previous_path_size-2];
              ref_yaw = atan2(ref_y - previous_ref_y, ref_x - previous_ref_x);

              spline_x_vals.push_back(previous_ref_x);
              spline_x_vals.push_back(ref_x);

              spline_y_vals.push_back(previous_ref_y);
              spline_y_vals.push_back(ref_y);
          }

          // Using 3 more future points for the spline calculation at a far distance.
          double car_next_d = 2 + 4 * car_lane;
          vector<double> fp0 = getXY(car_s + 30, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> fp1 = getXY(car_s + 60, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> fp2 = getXY(car_s + 90, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          spline_x_vals.push_back(fp0[0]);
          spline_y_vals.push_back(fp0[1]);

          spline_x_vals.push_back(fp1[0]);
          spline_y_vals.push_back(fp1[1]);

          spline_x_vals.push_back(fp2[0]);
          spline_y_vals.push_back(fp2[1]);

          // Adjusting the coordinates used to create the spline to the
          // coordinates and yaw of the car after consuming the previous path.
          for (int i = 0; i < spline_x_vals.size(); i++) {
            double shift_x = spline_x_vals[i] - ref_x;
            double shift_y = spline_y_vals[i] - ref_y;

            spline_x_vals[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            spline_y_vals[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline spline;
          spline.set_points(spline_x_vals, spline_y_vals);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Append points from the previous path in the next path for continuity.
          for (int i = 0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance y position on 30m ahead on x.
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double curr_x_disp = 0;
          for (int i = 1; i < 50 - previous_path_size; i++) {
            ref_vel += car_acceleration;
            if (ref_vel > MAX_SPEED) {
              ref_vel = MAX_SPEED;
            } else if (ref_vel < MAX_ACC_ENC) {
              ref_vel = MAX_ACC_ENC;
            }

            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = curr_x_disp + target_x/N;
            double y_point = spline(x_point);

            curr_x_disp = x_point;

            next_x_vals.push_back(x_point * cos(ref_yaw) - y_point * sin(ref_yaw) + ref_x);
            next_y_vals.push_back(x_point * sin(ref_yaw) + y_point * cos(ref_yaw) + ref_y);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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