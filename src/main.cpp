#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <cmath>
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

// Simple timer for measuring time
class Timer {
private:
  std::chrono::time_point<std::chrono::high_resolution_clock>  timestamp_start;
  double min = 0;
  double max = 0;
  double moving_avg = 0;
  unsigned int n_samples = 3;
  bool is_initialized = false;
public:

  Timer() {}
  ~Timer() {}

  void Start() {
    timestamp_start = std::chrono::high_resolution_clock::now();
  }

  double Stop() {
    std::chrono::duration<double> latency_tot = std::chrono::high_resolution_clock::now() - timestamp_start;
    double latency = latency_tot.count();
    if (is_initialized) {
      // Update min&max
      if (latency < min) {min = latency; }
      else if (latency > max) {max = latency; }
      // Update running average
      moving_avg -= moving_avg / n_samples;
      moving_avg += latency / n_samples;
    }
    else {
      min = latency;
      max=latency;
      moving_avg=latency;
      is_initialized = true;
    }
    return latency;
  }

  double getAverage() {return moving_avg;}
  double getMin() {return min;}
  double getMax() {return max;}
};


int main() {
  uWS::Hub h;
  Timer timer;

  // MPC is initialized here!
  MPC mpc;
  Eigen::VectorXd meas_previous = Eigen::VectorXd::Zero(6);

  h.onMessage([&mpc, &timer, &meas_previous](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    timer.Start();
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
          // Get car's current state
          const double px_0 = j[1]["x"];
          const double py_0 = j[1]["y"];
          const double psi_0 = j[1]["psi"];
          const double v_0 = 0.44704 *(double)j[1]["speed"]; // convert mph to meters/second
          const double delta_0 = -(double)j[1]["steering_angle"];  // Invert steering angle (delta)
          double a_0 = j[1]["throttle"];
          // Get xy way points from simulator
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // Calculate psid and acceleration by using previous measurements
          // Vector px, py, psi, v, delta, a
          Eigen::VectorXd meas_this(6);
          meas_this << px_0, py_0, psi_0, v_0, delta_0, a_0;
          Eigen::VectorXd meas_delta = meas_this - meas_previous;
          // angle needs special attention because of the jump from 2 * PI to 0
          meas_delta(2) = atan2(sin(meas_this(2) - meas_previous(2)), cos(meas_this(2) - meas_previous(2)));
          const double psid_0 = meas_delta(2);
          a_0 = meas_delta(3);
          meas_previous = meas_this;


          // Use static_latency to finetune latency. Especially in high speeds it's helpful
          const double static_latency = 0.0;
          const double latency = timer.getAverage() + static_latency;

          //
          // Predict car position after latency time
          //
          // yaw-rate, need to compensate because car won't react to steering angle changes ideally.
          const double f_steer = MPC::ds / (MPC::ds/MPC::ds_min + pow(v_0, 2));
          const double psid_1 = (1-f_steer)*psid_0 + f_steer*(v_0/MPC::Lf)*tan(delta_0);
          //const double psid_1 = psid_0;
          std::cout << "psid_1=" << psid_1 << std::endl;
          const double psi_1 = psi_0 + psid_1 * latency;
          const double v_1 = v_0 + a_0 * latency;
          // Depending of psid(yaw-rate) select straight line euqation or circle equation
          double px_1 = px_0;
          double py_1 = py_0;
          if (psid_1 < 0.001) {
            // Straight line model
            px_1 += v_0 * cos(psi_1) * latency;
            py_1 += v_0 * sin(psi_1) * latency;
          }
          else {
            // CTRV model
            px_1 += (v_0 / psid_1) * ( sin(psi_0 + psid_1 * latency) - sin(psi_0));
            py_1 += (v_0 / psid_1) * (-cos(psi_0 + psid_1 * latency) + cos(psi_0));
          }

          // Convert world coordinates to car-local coordinate system
          // Rotate 90-degrees to right so that car front is pointing
          // along x-axis --> This makes polynomial fit easier because
          // then we are looking for horizontallish line
          Eigen::VectorXd ptsx_transform_1(ptsx.size());
          Eigen::VectorXd ptsy_transform_1(ptsy.size());
          for (uint i=0; i < ptsx.size(); i++) {
              const double shift_x = ptsx[i] - px_1;
              const double shift_y = ptsy[i] - py_1;
              ptsx_transform_1(i) = (shift_x*cos(-psi_1) - shift_y*sin(-psi_1));
              ptsy_transform_1(i) = (shift_x*sin(-psi_1) + shift_y*cos(-psi_1));
          }

          //  Get 3rd order polynomial coefficients of given waypoints
          auto coeffs_1 = polyfit(ptsx_transform_1, ptsy_transform_1, 3);
          double cte_1 = polyeval(coeffs_1, 0);
          double epsi_1 = -atan(coeffs_1[1]);


          // State vector for MPC
          // px, py and psi are 0 because latency is already compensated in coeffs_1
          Eigen::VectorXd state(MPC::n_x_);
          state << 0, 0, 0, psid_1, v_1, cte_1, epsi_1;

          // Solve steering angle, throttle and predicted trajectory
          auto vars = mpc.Solve(state, coeffs_1);


          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 15;
          for (int i = 1; i < num_points; i++) {
              next_x_vals.push_back(poly_inc*i);
              next_y_vals.push_back(polyeval(coeffs_1, poly_inc*i));
          }


          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (uint i = 2; i < vars.size(); i++) {
            if(i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*MPC::Lf);
          msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*MPC::Lf);
          msgJson["throttle"] = vars[1];


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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
          this_thread::sleep_for(chrono::milliseconds(100));
          double lat = timer.Stop();
          std::cout << "Latency: loop=" << lat << " Avg=" << timer.getAverage() << std::endl;
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
