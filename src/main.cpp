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
#include "measurement_package.h"
#include "ukf.h"
#include "vehicle.h"

// for convenience
using json = nlohmann::json;


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
  UKF ukf = UKF();

  h.onMessage([&mpc, &timer, &ukf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
        // j[0] is a event type of message
        // j[1] is the json object which contains telemetry data
        string event = j[0].get<string>();
        if (event == "telemetry") {
          MeasurementPackage meas = getStateFromJSON(&j);
          printState(&meas.raw_measurements_, "RAW meas");
          ukf.ProcessMeasurement(meas);
          printState(&ukf.x_, "UKF Update");

          vector<Eigen::VectorXd> waypoints_global = getWaypointsFromJSON(&j);
          vector<Eigen::VectorXd> waypoints_trans = transformWaypoints(waypoints_global, ukf.x_(0), ukf.x_(1), ukf.x_(3));
          //std::cout << "Waypoints x\n" << waypoints_trans[0] << "Waypoints y\n"<< waypoints_trans[1] << std::endl;
          //  Get 3rd order polynomial coefficients of given waypoints
          auto coeffs_0 = polyfit(waypoints_trans[0], waypoints_trans[1], 3);

          // Calculate cte and epsi without effect of latency
          double cte_0 = polyeval(coeffs_0, 0);
          // Complete epsi equation for reference
          // double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[3] + 3 * coeffs[3] * pow(px, 2))
          // And it gets simplified to below equation becauce px=0 and psi=0
          double epsi_0 = -atan(coeffs_0[1]);

          // Here x,y and psi are zero because state is in car's coordinate
          // system
          Eigen::VectorXd state(6);
          const double latency = timer.getAverage();
          //const double latency = 0.1;
          Eigen::VectorXd x_latency = ukf.Prediction(latency);
          printState(&x_latency, "After Latency");
          const double delta_0 = ukf.x_(4);
          const double v_0 = ukf.x_(2);
          const double a_0 = ukf.x_(5);

          // Predict car position after latency time
          const double psi_1 = (v_0/VehicleModel::Lf) * delta_0 * latency;
          const double px_1 = v_0 * cos(delta_0) * latency;  // delta_0
          const double py_1 = v_0 * sin(delta_0) * latency;  // delta_0
          const double v_1 = v_0 + a_0 * latency;


          const double epsi_1 = epsi_0 + (v_0/VehicleModel::Lf) * delta_0 * latency;
          const double cte_1 = cte_0 + v_0 * sin(epsi_0) * latency;

          state << px_1, py_1, psi_1, v_1, cte_1, epsi_1;


          // Convert world coordinates to car-local coordinate system
          // Rotate 90-degrees to right so that car front is pointing
          // along x-axis --> This makes polynomial fit easier because
          // then we are looking for horizontallish line
          /*
          Eigen::VectorXd ptsx_transform_1(ptsx.size());
          Eigen::VectorXd ptsy_transform_1(ptsy.size());
          for (uint i=0; i < ptsx.size(); i++) {
              //const double shift_x = ptsx[i] - px_1;
              //const double shift_y = ptsy[i] - py_1;
              //ptsx_transform_1(i) = (shift_x*cos(-psi_1) - shift_y*sin(-psi_1));
              //ptsy_transform_1(i) = (shift_x*sin(-psi_1) + shift_y*cos(-psi_1));
              ptsx_transform_1(i) = ptsx_transform_0(i) + px_1;
              ptsy_transform_1(i) = ptsy_transform_0(i) + py_1;
          }
          */

          vector<Eigen::VectorXd> waypoints_1 = transformWaypoints(waypoints_global, x_latency(0), x_latency(1), x_latency(3));
          //  Get 3rd order polynomial coefficients of given waypoints
          auto coeffs_1 = polyfit(waypoints_1[0], waypoints_1[1], 3);



          // Solve steering angle, throttle and predicted trajectory
          auto vars = mpc.Solve(state, coeffs_1);



          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 20;
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
          msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*VehicleModel::Lf);
          msgJson["throttle"] = vars[1];
          //msgJson["throttle"] = 0.2;


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
