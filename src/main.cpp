#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "helper_functions.hpp"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;
  
  // MPC is initialized here!
  MPC mpc;
  int counter=0;
  h.onMessage([&mpc, &counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          /*
           * TODO: Calculate steering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
          
          
          //transform to car coordinates and fit
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          unityToCar(next_x_vals, next_y_vals, ptsx, ptsy, px, py,psi);
          Eigen::VectorXd xx(next_x_vals.size());
          Eigen::VectorXd yy(next_y_vals.size());
          for (int i=0;i<next_x_vals.size();++i) {
            xx[i]=next_x_vals[i];
            yy[i]=next_y_vals[i];
          }
          
          Eigen::VectorXd coef=polyfit(xx,yy,3);
          double cte=polyeval(coef, 0);
          double epsi = -std::atan(coef[1]);
          Eigen::VectorXd state(6); //x, y, psi, v, cte, epsi
          state << 0, 0, 0, v,cte,epsi;
          vector<double> result=mpc.Solve(state, coef);
          double steer_value=result[0]/deg2rad(25);
          double throttle_value=result[1];
          
          //Always accelerate with max value if driving very slow. THe MPC controller gets unstable at small velocities

          if (v<15)throttle_value=1.;
          int N=(result.size()-2)/2;
          for (int i=0;i<N;i++) {
            mpc_x_vals.push_back(result[2*i+2]);
            mpc_y_vals.push_back(result[2*i+2+1]);
          }
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          //Display the waypoints/reference line
          
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          
          //cout <<"CAR " <<px<<", "<<py<<", "<<psi<<std::endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(delay_steps*chrono::milliseconds(100));
          
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
