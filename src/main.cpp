#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  PID pid_throttle;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.11, 0.001, 2.0);// best manually
  pid_throttle.Init(0.4, 0, 0);

  // double dp[3] = {1, 1, 1}; // maybe adjust this value, need global
  double dp[3] = {0.001, 0.0001, 0.02};
  double tol = 0.1;// large enough will lead to twiddle inavailabe
  int times_twiddle = 0;
  bool plused = true;
  float best_err = std::numeric_limits<float>::max();
  double total_error = 0;
  bool dp_change = true;

  h.onMessage([&pid, &pid_throttle, &dp, &total_error,tol, &plused, &dp_change, &best_err, &times_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          // double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          std::cout << "angle" << angle << std::endl;
          double steer_value;
          double throttle;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // double *p = twiddle(pid, cte, 0.01 );
          // std::cout << "main p i d: " << pid.Kp << " " << pid.Ki << " " << pid.Kd << std::endl;
          // pid.Init(p[0], p[1], p[2]);
          // if (iteration % 100 == 0){
          //   dp[0] = 1;
          //   dp[1] = 1;
          //   dp[2] = 1;
          //   // adjust again
          // }
          pid.UpdateError(cte);
          // total_error += pow(cte,2);
          total_error += fabs(pid.TotalError());
          twiddle(pid, total_error, tol, dp, plused, dp_change, best_err, times_twiddle);
          // pid.UpdateError(cte);
          std::cout << "times_twiddle: " << times_twiddle << std::endl;
          std::cout << "after twiddle p i d: " << pid.Kp << " " << pid.Ki << " " << pid.Kd << std::endl;
          std::cout << "after twiddle dp: " << dp[0] << " " << dp[1] << " " << dp[2] << std::endl;
          std::cout << "best_err: " << best_err << std::endl;
          steer_value = pid.TotalError();
          std::cout << "steer_value: "<< steer_value << std::endl;

          // pid_throttle = twiddle(pid_throttle, 1/std::max(fabs(angle), 0.00001), tol, dp);
          // throttle = - pid_throttle.TotalError();
          // std::cout << "throttle" << throttle << std::endl;



          // // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          // msgJson["throttle"] = std::min(throttle, 0.3); // default 0.3;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
