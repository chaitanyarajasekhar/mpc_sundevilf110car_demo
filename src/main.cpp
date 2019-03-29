#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// #include "LinearMath/btMatrix3x3.h"
/***************** **********************************************************************/
class test
{
public:
  ros::NodeHandle nh;
  ros::Subscriber pose_sub;
  // ros::Publisher speed_pub, throt_pub, steer_pub;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;
  double roll, pitch , yaw;

  double x, z, x_prev, z_prev;
  std_msgs::Float64 speed, steer, throt;


  ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("commands/motor/speed",1);
  ros::Publisher throt_pub =  nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle",1);
  ros::Publisher steer_pub =  nh.advertise<std_msgs::Float64>("commands/servo/position",1);;

test(ros::NodeHandle &n){
  sub();
}

void sub(){
  pose_sub = nh.subscribe("vrpn_client_node/f110car_1/pose",10,&test::poseCallback,this);}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  pose.header = msg->header;
  pose.pose = msg->pose;
  quat = msg->pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

}

void publisher(double &throttle, double &steering){
    throt.data = throttle;
    steer.data = steering;

}};



/*************************************************************************************/

/*******************************************************************************************/

/*******************************************************************************************/

/*******************************************************************************************/
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }




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



int main(int argc, char **argv) {

  ros::init(argc,argv,"mpc_demo");
  ros::NodeHandle n;

  MPC mpc;
  test v1(n);

  v1.x_prev

  ros::Rate rate(2);

  while(ros::ok()){


          // vector<double> ptsx = j[1]["ptsx"];
          // vector<double> ptsy = j[1]["ptsy"];
          double px = v1.x;
          double py = v1.z;
          double psi = v1.pitch;
          // double v = j[1]["speed"];
          // double throttle_value = j[1]["throttle"];
          // double steer_value = j[1]["steering_angle"];

          //converting to miles per hour to meter per sec
          //v *= 0.44704;
          //steer_value *= -1;

          // vector<double> ptsx = j[1]["ptsx"];
          // vector<double> ptsy = j[1]["ptsy"];
          // double px = j[1]["x"];
          // double py = j[1]["y"];
          // double psi = j[1]["psi"];
          // double v = j[1]["speed"];
          // double throttle_value = j[1]["throttle"];
          // double steer_value = j[1]["steering_angle"];
          //
          // size_t i;
          //
          // /*
          // * TODO: Calculate steering angle and throttle using MPC.
          // *
          // * Both are in between [-1, 1].
          // *
          // */
          //
          // //Display the waypoints/reference line
          // //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // // the points in the simulator are connected by a Yellow line
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;
          //
          // //transform the waypoints to cars frame
          // Eigen::VectorXd x_waypoints(ptsx.size());
          // Eigen::VectorXd y_waypoints(ptsy.size());
          //
          // for (i =0; i<ptsx.size(); i++){
          //   double dx = ptsx[i] - px;
          //   double dy = ptsy[i] - py;
          //   x_waypoints[i] = dx * cos(-psi) - dy * sin(-psi);
          //   y_waypoints[i] = dx * sin(-psi) + dy * cos(-psi);
          //   next_x_vals.push_back(x_waypoints[i]);
          //   next_y_vals.push_back(y_waypoints[i]);
          // }
          // Eigen::VectorXd coeffs = polyfit(x_waypoints, y_waypoints, 3);
          // double cte = polyeval(coeffs, 0);
          // double epsi = -atan(coeffs[1]);
          // Eigen::VectorXd state(6);
          // state.fill(0.0);
          // // consider latency
          // double latency = 0.1;
          // const double Lf = 2.67;
          // state[0] = v * latency;
          // state[2] = v * latency * steer_value / Lf;
          // state[3] = v + throttle_value * latency;
          // state[4] = cte + v * sin(epsi) * latency;
          // state[5] = epsi + v * latency * steer_value / Lf;
          // //compute the actuator values
          // vector<double> result = mpc.Solve(state, coeffs);
          // steer_value = result[0];
          // throttle_value = result[1];
          // v1.publisher(result[0], result[1]);
          double a =0.2, b =0.7;
          v1.publisher(a,b);
          rate.sleep();
          ros::spinOnce();
        }

          return 0;
}

          //json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].


          //msgJson["steering_angle"] = - steer_value / deg2rad(25);
          //msgJson["throttle"] = throttle_value;

          /*
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;


          for (i = 2; i < result.size(); i ++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(result[i]);
            }
            else {
              mpc_y_vals.push_back(result[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

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
  */
// }
