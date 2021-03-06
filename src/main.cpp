#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "MPC.h"

// for convenience
using json = nlohmann::json;

const double STEER_MIN   = -25. * M_PI / 180; // minimal steering angle
const double STEER_MAX   =  25. * M_PI / 180; // maximal steering anlge
const double ACCEL_MIN   = -1.; // minimal acceleration (deceleration)
const double ACCEL_MAX   =  1.; // maximal acceleration
const double MPH_TO_MPS  =  1.609344 /* mi/km */ / 3600 /* hr/sec */ * 1000 /* km/m */;
const double V_REFERENCE =  110 * MPH_TO_MPS; // speed limit
const double LATENCY     =  0.1;  // default latency in secs
const double DT          =  0.25; // delta time in sec
const int    POLYORDER   =  3;    // fitting a 3rd order polynom
const int    N           =  10;   // number of iterations

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
   auto found_null = s.find("null");
   auto b1 = s.find_first_of("[");
   auto b2 = s.rfind("}]");
   if (found_null != string::npos)
      return "";
   else if (b1 != string::npos && b2 != string::npos)
      return s.substr(b1, b2 - b1 + 2);
   return "";
}

int main()
{
   uWS::Hub h;

   // MPC is initialized here
   MPC mpc(N, DT, LATENCY, POLYORDER, STEER_MIN, STEER_MAX, ACCEL_MIN, ACCEL_MAX, V_REFERENCE);

   h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
   {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      string sdata = string(data).substr(0, length);
      cout << sdata << endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
      {
         string s = hasData(sdata);
         if (s != "")
         {
            auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event == "telemetry")
            {
               // j[1] is the data JSON object
               vector<double> ptsx = j[1]["ptsx"];  // middle x coords
               vector<double> ptsy = j[1]["ptsy"];  // middle y coords
               double px = j[1]["x"];               // x world coord.
               double py = j[1]["y"];               // y world coord.
               double psi = j[1]["psi"];            // current heading angle
               double v = double(j[1]["speed"]) * MPH_TO_MPS; // current speed in m/s
               double delta0 = double(j[1]["steering_angle"]) * -1; // current steering angle in radians - inverting
               double a0 = j[1]["throttle"];        // current throttle value [-1, 1]
                                
               // Calculate steering angle and throttle using MPC.
               mpc.preprocess(ptsx, ptsy, px, py, psi, v, delta0, a0);
               auto vars = mpc.solve();
               // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
               // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].
               double steer_value = vars[0] / STEER_MAX;
               double throttle_value = vars[1];
                                
               // Display the MPC predicted trajectory 
               vector<double> mpc_x_vals; mpc_x_vals.reserve(N - 1);
               vector<double> mpc_y_vals; mpc_y_vals.reserve(N - 1);
               for (unsigned short n = 0; n < N - 2; ++n)
               {
                  mpc_x_vals.push_back(vars[2 * n + 2]);
                  mpc_y_vals.push_back(vars[2 * n + 3]);
               }
                                
               json msgJson;
               msgJson["steering_angle"] = -steer_value; // inverting orientation
               msgJson["throttle"] = throttle_value;
                                
               //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
               // the points in the simulator are connected by a Green line
               msgJson["mpc_x"] = mpc_x_vals;
               msgJson["mpc_y"] = mpc_y_vals;
                                
               //Display the waypoints/reference line
               vector<double> next_x_vals = mpc.ptsx_car_;
               vector<double> next_y_vals(mpc.ptsy_car_.size());
               for (unsigned short n = 0; n < mpc.ptsy_car_.size(); ++n)
                  next_y_vals[n] = mpc.polyeval(mpc.ptsx_car_[n]);
                                
               //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
               // the points in the simulator are connected by a Yellow line
               msgJson["next_x"] = next_x_vals;
               msgJson["next_y"] = next_y_vals;
                                
               auto msg = "42[\"steer\"," + msgJson.dump() + "]";
               std::cout << msg << std::endl;
                
               // Latency
               // The purpose is to mimic real driving conditions where
               // the car does actuate the commands instantly.
               // Feel free to play around with this value but should be to drive
               // around the track with 100ms latency.
               this_thread::sleep_for(chrono::milliseconds((int)LATENCY*1000));
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
         }
         else
         {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
      }
   });

   // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
   h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t)
   {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1)
         res->end(s.data(), s.length());
      else // i guess this should be done more gracefully?
         res->end(nullptr, 0);

   });

   h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
   {
      std::cout << "Connected!!!" << std::endl;
   });

   h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length)
   {
      ws.close();
      std::cout << "Disconnected" << std::endl;
   });

   int port = 4567;
   if (!h.listen(port))
   {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
   }

   std::cout << "Listening to port " << port << std::endl;

   h.run();
}
