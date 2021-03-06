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

// converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// convert a set of (x_map,y_map) values in map FoR, to the vehicle local FoR, given the
// cars position and heading (xc, yc, psi)
std::pair<std::vector<double>, std::vector<double>> Map2VehicleCoord(
    vector<double> x_map, vector<double> y_map, double xc, double yc, double psi){

  vector<double> x_local, y_local;

  for (size_t i = 0; i != x_map.size(); i++) {
    x_local.push_back( (x_map[i] - xc) * cos(psi) + (y_map[i] - yc) * sin(psi) );
    y_local.push_back( (y_map[i] - yc) * cos(psi) - (x_map[i] - xc) * sin(psi) );
  }

  return std::make_pair(x_local, y_local);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // use latency state prediction
  bool lat_predict = false;

  h.onMessage([&mpc, lat_predict](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // reference trajectory in MAP FoR
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // state vector variables in MAP FoR
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.44704; // mph to m/s (SI)

          // latest steering and throttle inputs processed by the car
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // latency correction on state
          long latency = 100; // in ms
          if (lat_predict){
            double dt_lat = latency / 1000; // in seconds
            double Lf = 2.67;
            px += v * cos(psi) * dt_lat;
            py += v * sin(psi) * dt_lat;
            psi += - v * steer_value / Lf * dt_lat;
            v += throttle_value * dt_lat;
          }

          // state vector variables in LOCAL FoR
          double px_local = 0;
          double py_local = 0;
          double psi_local = 0;
          double v_local = v;

          // reference trajectory in LOCAL FoR
          auto waypoints_local = Map2VehicleCoord(ptsx, ptsy, px, py, psi);
          vector<double> ptsx_local = waypoints_local.first;
          vector<double> ptsy_local = waypoints_local.second;

          // for convenience, THE OPTIMIZATION PROBLEM WILL BE RESOLVED IN LOCAL FRAME OF REFERENCE

          // cast the std::vectors to Eigen::VectorXd
          Eigen::Map<Eigen::VectorXd> ptsx_vxd(&ptsx_local[0], ptsx_local.size());
          Eigen::Map<Eigen::VectorXd> ptsy_vxd(&ptsy_local[0], ptsy_local.size());

          // get coefficients of degree 3 polynomial fit of the trajectory
          auto coeffs = polyfit(ptsx_vxd, ptsy_vxd, 3);

          // cross track error calculated by evaluating polynomial at x, f(x) and subtracting y.
          double cte = polyeval(coeffs, px_local) - py_local;

          // orientation error: psi - desired_psi
          // where desired_psi can be calculated as the tangential angle of the polynomial f evaluated at x
          // using expression for tangential angle of a 3rd degree polynomial:
          double epsi = psi_local - atan(coeffs[1] + 2 * coeffs[2] * px_local + 3 * coeffs[3] * pow(px_local, 2));

          // build state input for the solve function
          Eigen::VectorXd state(6);
          state << px_local, py_local, psi_local, v_local, cte, epsi;

          // optimization
          vector<double> solution = mpc.Solve(state, coeffs);

          // extract optimal actuator inputs
          // change steering angle sign for consistency: if delta is  positive we rotate counter-clockwise,
          // or turn left. In the simulator however, a positive value implies a right turn and
          // a negative value implies a left turn
          steer_value = - solution[0] / deg2rad(25); // normalized steer_value to [-1,1]
          throttle_value = solution[1];

          // extract optimal predicted trajectory
          int N = int((solution.size() - 2) / 2);
          // + 1 in start pointers just to exclude current state (0 index) from visualization of predicted line
          vector<double> x_sol(&solution[2 + 1], &solution[2 + N]);
          vector<double> y_sol(&solution[2 + N + 1], &solution[2 + N * 2]);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // MPC predicted trajectory in vehicle's local coordinate system
          vector<double> mpc_x_vals = x_sol;
          vector<double> mpc_y_vals = y_sol;

          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // waypoints/reference line in vehicle's local coordinate system
          vector<double> next_x_vals = ptsx_local;
          vector<double> next_y_vals = ptsy_local;

          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // the purpose is to mimic real driving conditions where the car doesn't actuate the commands instantly.
          // for the exercise, the car should be able to drive around the track
          // with a latency of 100 ms

          this_thread::sleep_for(chrono::milliseconds(latency));
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
