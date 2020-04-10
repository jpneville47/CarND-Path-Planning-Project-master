#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

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


  // initial lane
  double target_lane = 1; //Note -> 2 + 4*target_lane for d in Frenet

  // initial velocity
  double target_speed = 0.0; //22.12 [m/s] Note -> [mph] / 2.237 = [m/s] 
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &target_lane, &target_speed]
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

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int next_point;

          next_point = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

          double head_way;

          //***************************************************************//

          // Sense the world around you

          // Variables for the statemachine
          bool slow_down = false;


          // 6945 is the default maximum dsitance to represent when no actor is occupying a lane
          double next_actor_station_0 = 6945;
          double next_actor_station_1 = 6945;
          double next_actor_station_2 = 6945;

          double prev_actor_station_0 = 6945;
          double prev_actor_station_1 = 6945;
          double prev_actor_station_2 = 6945; 

          double next_actor_velocity_0;
          double next_actor_velocity_1;
          double next_actor_velocity_2;

          double prev_actor_velocity_0 = 0;
          double prev_actor_velocity_1 = 0;
          double prev_actor_velocity_2 = 0;

          double val_0 = 0;
          double val_1 = 0;
          double val_2 = 0;

          for(int i = 0; i < sensor_fusion.size(); ++i){
            double actor_station = sensor_fusion[i][5];
            double actor_lane = sensor_fusion[i][6];

            double actor_vx = sensor_fusion[i][3];
            double actor_vy = sensor_fusion[i][4];
            double actor_velocity = sqrt(actor_vx*actor_vx + actor_vy*actor_vy);

            double relative_distance = actor_station - car_s;
            double relative_lane = actor_lane - car_d;

            // std::cout << "s value: " << relative_distance << " \td value: " << relative_lane << "\tActor Velocity: " << actor_velocity << "\n";

            if(relative_distance > 0 && relative_distance < 30 && abs(relative_lane) < 2){
              head_way = actor_velocity;
              slow_down = true;
            }

            if(relative_distance > 0){
              
              
              if(actor_lane < 4 && relative_distance < next_actor_station_0){
                next_actor_station_0 = relative_distance;
                next_actor_velocity_0 = actor_velocity;

                val_0 = relative_distance + actor_velocity;
              }

              if(next_actor_station_0 > 500){
                val_0 = 500;
              }


              if(actor_lane >= 4 && actor_lane <= 8 && relative_distance < next_actor_station_1){
                next_actor_station_1 = relative_distance;
                next_actor_velocity_1 = actor_velocity;

                val_1 = relative_distance + actor_velocity;
              }

              if(next_actor_station_1 > 500){
                val_1 = 500;
              }

              if(actor_lane > 8 && relative_distance < next_actor_station_2){
                next_actor_station_2 = relative_distance;
                next_actor_velocity_2 = actor_velocity;

                val_2 = relative_distance + actor_velocity;
              }

              if(next_actor_station_2 > 500){
                val_2 = 500;
              }

            }

            if(relative_distance <= 0){

              // Lane 0
              if(actor_lane < 4 && abs(relative_distance) < abs(prev_actor_station_0)){
                prev_actor_station_0 = relative_distance;
                prev_actor_velocity_0 = actor_velocity;
              }

              // Lane 1
              if(actor_lane >= 4 && actor_lane <= 8 && abs(relative_distance) < abs(prev_actor_station_1)){
                prev_actor_station_1 = relative_distance;
                prev_actor_velocity_1 = actor_velocity;
              }

              // Lane 2
              if(actor_lane > 8 && abs(relative_distance) < abs(prev_actor_station_2)){
                prev_actor_station_2 = relative_distance;
                prev_actor_velocity_2 = actor_velocity;
              }


            }

          }

          // Diagnostic output for evaluating the heuristic
          // std::cout << "Next Actor 0: " << next_actor_station_0 << "\tNext Actor Velocity 0: " << next_actor_velocity_0 << "\tVal 0: " << val_0 << "\tPrev Actor Station: " << prev_actor_station_0<< "\n";
          // std::cout << "Next Actor 1: " << next_actor_station_1 << "\tNext Actor Velocity 1: " << next_actor_velocity_1 << "\tVal 1: " << val_1 << "\tPrev Actor Station: " << prev_actor_station_1<< "\n";
          // std::cout << "Next Actor 2: " << next_actor_station_2 << "\tNext Actor Velocity 2: " << next_actor_velocity_2 << "\tVal 2: " << val_2 << "\tPrev Actor Station: " << prev_actor_station_2<< "\n";

          if(slow_down){
            if(target_speed > head_way){
              target_speed -= 0.05;
            }
          }
          else if(target_speed <= 22){
            target_speed += 0.1;
          }

          double var_out = 2+4*target_lane;

          if(car_d > (1+4*target_lane) && car_d < (3+4*target_lane)){


            switch(int(target_lane)){
              case 0:
                
                if(target_lane == 0 && val_1 > 1.2*val_0 && abs(prev_actor_station_1) > 7 && target_speed > prev_actor_velocity_1){
                  target_lane = 1;
                }

                break;

              case 1:
                
                if(target_lane == 1 && val_0 > 1.2*val_1 && abs(prev_actor_station_0) > 7 && target_speed > prev_actor_velocity_0){
                  target_lane = 0;
                }

                if(target_lane == 1 && val_2 > 1.2*val_1 && abs(prev_actor_station_2) > 7 && target_speed > prev_actor_velocity_2){
                  target_lane = 2;
                }

                break;
              
              case 2:
                
                if(target_lane == 2 && val_1 > 1.2*val_2 && abs(prev_actor_station_1) > 7 && target_speed > prev_actor_velocity_1){
                  target_lane = 1;
                }
              
                break;
            }
          }

          // Spline generation influenced by the work of Aaron Brown as presented in the Udacity Path Planning Q&A
          // Initialize array for spline
          std::vector<double> X, Y;
          double reference_x = car_x;
          double reference_y = car_y;
          double reference_yaw = deg2rad(car_yaw);

          int previous_size = previous_path_x.size();

          // Note -> car_yaw is reported in degrees! Convert to radians using deg2rad(car_yaw)

          // Calculate the cars current and previous position to get the direction of travel
          if(previous_size < 2){
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);

            // By putting a point tangent to the car's direction of travel and the current point we ensure that the spline initializes a perfect initial trajectory
            X.push_back(previous_car_x);
            X.push_back(car_x);

            Y.push_back(previous_car_y);
            Y.push_back(car_y);

          }

          else{
            // Use the previous path
            reference_x = previous_path_x[previous_size-1];
            reference_y = previous_path_y[previous_size-1];

            double reference_x_prev = previous_path_x[previous_size-2];
            double reference_y_prev = previous_path_y[previous_size-2];
            reference_yaw = atan2(reference_y - reference_y_prev, reference_x - reference_x_prev);

            X.push_back(reference_x_prev);
            X.push_back(reference_x);

            Y.push_back(reference_y_prev);
            Y.push_back(reference_y);

          }

          // Get 3 points into the future using the Frenet frame specifing the desired lane
          // output as points in the global frame

          vector<double> next_waypoint_0 = getXY(car_s + 40, (2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s + 60, (2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s + 90, (2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          X.push_back(next_waypoint_0[0]);
          X.push_back(next_waypoint_1[0]);
          X.push_back(next_waypoint_2[0]);

          Y.push_back(next_waypoint_0[1]);
          Y.push_back(next_waypoint_1[1]);
          Y.push_back(next_waypoint_2[1]);

          // Convert global xy points into ego frame to build the spline from
          for(int i = 0; i < X.size(); ++i){
            double shift_x = X[i] - reference_x;
            double shift_y = Y[i] - reference_y;

            X[i] = (shift_x * cos(0-reference_yaw) - shift_y * sin(0-reference_yaw));
            Y[i] = (shift_x * sin(0-reference_yaw) + shift_y * cos(0-reference_yaw));
          }

          // Build spline and re-sample so that the distance between each point achieves the target speed
          
          tk::spline s;
          s.set_points(X,Y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Populate with previous points
          for(int i = 0; i < previous_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Pick an x value a certain distance from the host vehicle to evaluate a triangle
          double triangle_x = 20.0;
          double triangle_y = s(triangle_x);
          // Determine the length between the host pose and the xy value on the spline at the target x value
          double target_dist = sqrt(triangle_x*triangle_x + triangle_y*triangle_y);

          double x_step = 0;

          for(int i = 0; i <= 30 - previous_path_x.size(); ++i){

            // Determine the number of points that the distance should be discretized into such that when each point is evaluated at a rate of 20 ms the car will travel at the desired target speed
            double Num_Points = (target_dist/(0.02*target_speed));
            double x_point = x_step + triangle_x/Num_Points;
            double y_point = s(x_point);

            x_step = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Convert the spline back into the global frame and push to next_x and next_y
            x_point = (x_ref * cos(reference_yaw) - y_ref*sin(reference_yaw));
            y_point = (x_ref * sin(reference_yaw) + y_ref*cos(reference_yaw));

            x_point += reference_x;
            y_point += reference_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }


          //**************************************************************************//

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