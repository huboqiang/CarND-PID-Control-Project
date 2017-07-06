#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double init_p = -0.08;
double init_i =  0.0;
double init_d =  -20;
double target_speed =  30.0;

void usage()
{
  std::cout << "./pid -p -0.08 -i 0.00 -d -20 -s 30" << std::endl;
  std::cout << "   -p <float>   init_p, default = " << init_p << std::endl;
  std::cout << "   -i <float>   init_i ,default = " << init_i << std::endl;
  std::cout << "   -d <float>   init_d ,default = " << init_d << std::endl;
  std::cout << "   -s <float>   target_speed ,default = " << target_speed << std::endl;
  std::cout << "   -h           get help information" << std::endl;
  exit(0);
}


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;


  int c;
  while((c=getopt(argc, argv, "p:i:d:s:h")) !=-1) {
  		switch(c) {
        case 'p': init_p = atof(optarg); break;
        case 'i': init_i = atof(optarg); break;
        case 'd': init_d = atof(optarg); break;
        case 's': target_speed = atof(optarg); break;
        case 'h':             usage();   break;
        default: usage();
      }
  }


  PID pid_steer;
  PID pid_throttle;
  // TODO: Initialize the pid variable.
  pid_steer.Init(init_p, init_i, init_d);
  pid_throttle.Init(-0.1, 0.0, -0.1);

  h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.GetUpdateValue();

          double ts = target_speed;
          if(fabs(cte) > 1.0){
            ts = target_speed * (1-fabs(cte)/5.0);
            if(ts < 0){
              ts = 3;
              steer_value *= 2;
            }
          }
          double cte_speed = speed - ts;
          pid_throttle.UpdateError(cte_speed);
          throttle = pid_throttle.GetUpdateValue();

          //std::cout << "Time:" << pid_steer.timeI << std::endl;
          //std::cout << "CTE: " << cte << std::endl;
          //std::cout << "Speed: " << speed << std::endl;
          //std::cout << "SteerValue: " << steer_value << std::endl;
          //std::cout << "Angle: " << angle << std::endl;

          std::cout << pid_steer.timeI << "\t" << cte << "\t" << speed;
          std::cout << "\t" <<  steer_value << "\t" << angle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          //std::cout << "angle = " << angle << std::endl;
          //std::cout << "steer = " << rad2deg(steer_value) << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
