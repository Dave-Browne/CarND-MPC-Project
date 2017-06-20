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

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object will be returned in string format,
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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //std::cout << "---------------------\n";
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Reference trajectory in global coords
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          // Actual vehicle characteristics in global coords
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          double cte, epsi, derivative;
          int order;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          * The JSON object sent back from the simulator command server:
          *  Fields:
          *   ptsx (Array) - The global x positions of the waypoints.
          *   ptsy (Array) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          *   psi (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions.
          *   psi_unity (float) - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
          *   x (float) - The global x position of the vehicle.
          *   y (float) - The global y position of the vehicle.
          *   steering_angle (float) - The current steering angle in radians.
          *   throttle (float) - The current throttle value [-1, 1].
          *   speed (float) - The current velocity in mph.
          */
          // Convert v from mph to m/s
          // Simulator sends velocity in mph and waypoints are given in m
          v = v * 1600 / 3600;

          // Change steering angle to allow for counter steering
          steer_value *= -1;

          // Accomodate for latency - predict px, py, psi & v at time latency
          double latency = 0.1;
          double Lf = 2.67;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi += v * steer_value * latency / Lf;
          v += throttle_value * latency;

          // Translate the ref traj from global to vehicle coords
          // Vehicle is at (px, py). To change to vehicle coords subtract px from x and py from y
          // Rotate the axis so that x is in direction car is facing, y is 90deg to the left.
          for (size_t i=0; i<ptsx.size(); ++i) {
            double xn = ptsx[i] - px;
            double yn = ptsy[i] - py;
            ptsx[i] = xn * cos(psi) + yn * sin(psi);
            ptsy[i] = yn * cos(psi) - xn * sin(psi);
          }
          px -= px;
          py -= py;
          psi -= psi;

          // Convert ptsx from std::vector to Eigen::vector for use in polyfit
          Eigen::Map<Eigen::VectorXd> ptsx_(ptsx.data(), ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_(ptsy.data(), ptsy.size());

          // Find the polynomial coefficients for the reference trajectory
          // Polynomial of 3rd order: f(x) = Ax^3 + Bx^2 + Cx + D
          //             Derivative: f'(x) = 3Ax^2 + 2Bx + C
          order = 3;
          auto coeffs = polyfit(ptsx_, ptsy_, order);

          // Find cte, the distance of vehicle from the ref traj.
          // This is the difference in y-values (y_ref_traj - y_vehicle)
          cte = polyeval(coeffs, px) - py;

          // Find epsi, the difference between vehicle orientation and ref traj orientation.
          derivative = 3*coeffs[3]*px*px + 2*coeffs[2]*px + coeffs[1];
          epsi = psi - atan(derivative);

          // State vector
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          // Run the MPC controller
          auto mpc_traj = mpc.Solve(state, coeffs);

          // N is the number of (x, y) points in the mpc trajectory
          int N = (mpc_traj.size()-2) / 2;

          // Extract the steering angle and acceleration from the latency-compensated MPC controller.
          steer_value = -mpc_traj[2*N] / deg2rad(25);
          throttle_value = mpc_traj[2*N+1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i=0; i<N; ++i) {
            mpc_x_vals.push_back(mpc_traj[i]);
            mpc_y_vals.push_back(mpc_traj[N+i]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (size_t i=1; i<ptsx.size(); ++i) {
            next_x_vals.push_back(ptsx[i]);
            next_y_vals.push_back(ptsy[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car doesn't actuate the commands instantly.
          //
          // Feel free to play around with this value, but it should be
          // possible to drive around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
  // program doesn't compile :-(
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
