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

//set this to add a latency in the system
double latency_s = 0.1;  
double timeScale = 1; //Makes the simulator run x times faster, 1 is normal speed
bool do_twiddle = false;



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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  double old_v=0;
  

  bool firstTime= timeScale!=1; // only if we use timescale

//&do_twiddle, &timeScale
  h.onMessage([&mpc, &old_v,&firstTime](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          auto start_time = std::chrono::high_resolution_clock::now();
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double Lf=2.67;
          //cout << px << ":" << py << endl;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          * From the QA video:
          */
         //Change the coordinate system of the map to that of the car
          for(int i = 0; i < ptsx.size(); i++){
              double shift_x = ptsx[i]-px;
              double shift_y = ptsy[i]-py;
              
              ptsx[i] = (shift_x*cos(-psi)-shift_y*sin(-psi));
              ptsy[i] = (shift_x*sin(-psi)+shift_y*cos(-psi));
          }

          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,6);

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,6);

          auto coeffs = polyfit(ptsx_transform,ptsy_transform,3);

          double cte = polyeval(coeffs,0);
          double epsi = -atan(coeffs[1]);

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];// almost accelration

          Eigen::VectorXd state(6);
          // The lectures mentioned that throttle_value should be good enough 
          // Don't know if this is better, should try it a bit!
          double accelration1 =(v-old_v);
          double accelration2 =throttle_value*latency_s;
          double accelration = accelration2;//(accelration1+accelration2)/2 ;
          double v_t1 = v+accelration; 
          old_v=v;
          double average_v = ((v_t1+v) / 2);
          double psi_t1 = (v_t1 / Lf) * -steer_value * (latency_s); //change in psi
          double x_t1 = average_v * cos(psi_t1/2) * (latency_s); // due to a, v will be different at t and t+1, so average ((v_latency+v)/2) should be more accurate
          double y_t1 = average_v * sin(psi_t1/2) * (latency_s); 
          double cte_t1 = cte + average_v * sin(psi_t1/2) * (latency_s); // Lesson 18:10 Errors
          double epsi_t1 = epsi + (v_t1 / Lf) * -steer_value * (latency_s); 
          state << x_t1, y_t1, psi_t1, 
                    v_t1, cte_t1, epsi_t1;

          // calculate steering angle and throttle
          auto vars = mpc.Solve(state,coeffs);
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);//steer_value;
          msgJson["throttle"] = vars[1];//throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          for(int i = 2; i<vars.size(); i++){
            if(i%2 == 0){
              mpc_x_vals.push_back(vars[i]);
            }else{
              mpc_y_vals.push_back(vars[i]);
            }
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;
          for(int i = 1; i<num_points; i++){
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs,poly_inc*i));
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          //Twiddle to get good parameters (Dont work with the standard simulator!)
          if(firstTime || do_twiddle  ){
            if(firstTime || mpc.twiddle(v,px, py, cte)  ){
              firstTime=false;
              json msgJson2;
              msgJson2["timeScale"] = timeScale;
              msg = "42[\"reset\"," + msgJson2.dump() + "]";
            }
          }

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
          

          auto end_time = std::chrono::high_resolution_clock::now();
          auto time_to_process_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
          //cout << time_to_process_ms << "ms" <<  endl;
          if(timeScale==1) time_to_process_ms = 0; // only do this when running faster
          //The latency is reduced when running simulation faster, as when going double speed 50 ms = 100 ms etc etc.
          // And also reduce the "time to process" due to this is more significant when going fast.
          
          //cout << "time to wait: " << int((latency_s*1000-time_to_process_ms)/timeScale) << " = " << int((latency_s*1000-time_to_process_ms)) <<  " timescale: " << timeScale << " process time: " << time_to_process_ms << endl;
          int delay_time = int((latency_s*1000-time_to_process_ms)/timeScale);
          this_thread::sleep_for(chrono::milliseconds(delay_time)); //This is 100 ms when timescale is 1
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
    //std::cout << "Connected!!!" << std::endl; //This is annoying if using reset
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
