#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>




// for convenience
using json = nlohmann::json;

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

inline double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
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

  PID pid;
  double init_Kp;
  double init_Ki;
  double init_Kd;

  long count_section = 0;

  // TODO: Initialize the pid variable with default values if not given as parameter.
  if (argc == 1) {
  	cout << "one parameter" << endl;
  	init_Kp = 1.09;
  	init_Ki = 0;
  	init_Kd = 27;
  }
  else if (argc == 4) {
  	init_Kp = atof(argv[1]);
  	init_Ki = atof(argv[2]);
  	init_Kd = atof(argv[3]);
  }
  else {
  	cerr << "ERROR in input parameters. USAGE1: pid USAGE2: pid Kp Ki Kd" << endl;
    return -1;
  }
  cout << "Load values --- Kp= " << init_Kp << " , Ki= " << init_Ki << " , Kd= " << init_Kd
  		<< endl;
  pid.Init(init_Kp, init_Ki, init_Kd);

  h.onMessage([&pid, &count_section](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value = pid.GetSteerInput();
          //see aux function above to clip the steer_value between MIN (-1) and MAX (+1) 
          steer_value = clip(steer_value, -1.0, +1.0);

          // DEBUG
          //std::cout << "ITER " << pid.iter << ", CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          pid.accum_error += fabs(cte);
          if (pid.iter == LENGTH_TOTAL_ERR_SAMPLES) {
          	cout << "ACCUMULATED ERROR AFTER " << LENGTH_TOTAL_ERR_SAMPLES << " SAMPLES: " << pid.accum_error << endl;
          }
          if (pid.iter > 0 && pid.iter % LENGTH_AVERAGE_SAMPLES == 0) {
          	count_section +=1;
          	cout << "LEG " << count_section << ": AVERAGE ERROR OVER LAST " << LENGTH_AVERAGE_SAMPLES << " SAMPLES:" \
          		<< pid.average_error << endl;
          	pid.average_error = 0;
          } else {
          	pid.average_error += fabs(cte) / LENGTH_AVERAGE_SAMPLES;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          pid.iter += 1;
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
