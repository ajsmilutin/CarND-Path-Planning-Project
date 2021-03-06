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
#include "Polynomial.h"
#include "PathPlanner.h"
//#include "SpeedUp.h"
#include "PathHelpers.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.


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




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  PathHelpers pathHelper;

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
    pathHelper.push_back(x, y, s, d_x, d_y);
  }

  double kmh = 1/3.6;  // 1 m/s = 3.6 kmh
  double mph = 1.6 * kmh;  // 1 mph = 1.6kmh;
  double max_speed = 47.5 * mph;
  std::cout<<"Max speed :"<<max_speed<<std::endl;
  double max_acceleration = 8;
  PathPlanner pathPlanner(max_speed, max_acceleration);
  pathPlanner.SetPathHelper(pathHelper);
  pathPlanner.setLane(1);
 // SpeedUp speedUp(max_speed, max_acceleration);
  //speedUp.setLane(1);


  h.onMessage(
      [&pathPlanner](
          uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
              Eigen::VectorXd carState(6);
              carState<< j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"];


              // Previous path data given to the Planner
              auto previous_path_x = j[1]["previous_path_x"];
              auto previous_path_y = j[1]["previous_path_y"];
              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];

              vector<double> next_x_vals;
              vector<double> next_y_vals;

              map<int, vector<double>> fusion_data;

              for (auto i=sensor_fusion.begin(); i<sensor_fusion.end(); i++){
                int id = *(i->begin());
                vector<double> data;
                for (auto j = i->begin()+1; j!=i->end(); j++)
                  data.push_back(*j);
                fusion_data[id]=data;
              }

              pathPlanner.setCarPosition(carState);
              pathPlanner.setPreviousPath(previous_path_x, previous_path_y);
              pathPlanner.setFusionData(fusion_data);

              pathPlanner.nextXY(next_x_vals, next_y_vals);

              json msgJson;

              // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
              msgJson["next_x"] = next_x_vals;
              msgJson["next_y"] = next_y_vals;

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
















































































