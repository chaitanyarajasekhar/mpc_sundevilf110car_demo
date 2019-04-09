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

// double initial =0 , final = 0, init_time = 0 , final_time = 0;

class test
{
public:
  ros::NodeHandle nh;
  ros::Subscriber pose_sub;
  // ros::Publisher speed_pub, throt_pub, steer_pub;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;
  double roll, pitch , yaw;
  double current_x, current_z, prev_x, prev_z, current_psi, current_time, previous_time;
  double current_throttle, current_steering;

  vector<double> traj_x, traj_y;

  std_msgs::Float64 speed, steer, throt;
  ros::Time timenow = ros::Time::now() ;
  ros::Time time_var;



  ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("commands/motor/speed",1);
  ros::Publisher throt_pub =  nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle",1);
  ros::Publisher steer_pub =  nh.advertise<std_msgs::Float64>("commands/servo/position",1);;

test(ros::NodeHandle &n){
  sub();
}

void sub()
{
  pose_sub = nh.subscribe("vrpn_client_node/f110car_1/pose",10,&test::poseCallback,this);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  time_var = msg->header.stamp;
  pose.header = msg->header;
  pose.pose = msg->pose;
  // ROS_INFO("pose_x = [%f]", pose.pose.position.x);
  quat = msg->pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);


}

void publisher(double &throttle, double &steering)
{

    throt.data = throttle;
    steer.data = steering;
    current_throttle = throttle;
    current_steering = steering;
    ROS_INFO("throttle = [%f], steering = [%f]", throttle, steering);
    throt_pub.publish(throt);
    steer_pub.publish(steer);


}

void break_publisher(){
  speed.data = 0;
  speed_pub.publish(speed);
}

double pose_(){
  return pose.pose.position.x;
}

ros::Time getTime(){
  return ros::Time::now();
}
};



/*************************************************************************************/

/*******************************************************************************************/

/*******************************************************************************************/

/*******************************************************************************************/
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double round(double var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =37.6716    for rounding off value
    // then type cast to int so value is 3766
    // then divided by 100 so the value converted into 37.66
    double value = (int)(var * 100 + .5);
    return (double)value / 100;
}


//
//
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
  //assert(order >= 1 && order <= xvals.size() - 1);
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

  vector<double> traj_x_init, traj_y_init;

  for (size_t j = 0; j < 100; j++){
    traj_x_init.push_back(-3.63+.029*j);
    traj_y_init.push_back(0.48);
    // ROS_INFO("traj_x,traj_y = [%f , %f]",traj_x_init[j],traj_y_init[j]);
  }

  // const vector<double> traj_x = traj_x_init, traj_y = traj_y_init;
  v1.traj_x = traj_x_init;
  v1.traj_y = traj_y_init;

  ROS_INFO("traj_x_size = [%d]", v1.traj_x.size());

  bool initial_cond = true;
  int initial_empty_ros_spin = 10;

  int r = 10;


  ros::Rate rate(r);
  // int cnt = 0;
  while(ros::ok()){
    // cnt++;
    // if (cnt > 12)
    //   break;

    v1.current_x = round(v1.pose.pose.position.x);
    v1.current_z = round(v1.pose.pose.position.z);
    v1.current_psi = v1.pitch;
    v1.current_time = v1.time_var.nsec;
    // ROS_INFO("v1_c_x = [%f] v1_c_y = [%f]  ", v1.current_x, v1.current_z);
    // ROS_INFO(" trajectory size while loop starting [%ld]", v1.traj_x.size());


    // v1.current_time = v1.timenow.nsec;
    // ROS_INFO("time = [%f] ",v1.current_time);

    // initial_empty_ros_spin using this so that motion capture gets the value in v1.current_x taking 5 spins
    if (initial_cond == true || initial_empty_ros_spin > 0){
      initial_cond = false;
      initial_empty_ros_spin = initial_empty_ros_spin -1;
      double a =0.0, b = 0.5;
      v1.publisher(a,b);
      // ROS_INFO("traj_x_size in if block= [%d]", v1.traj_x.size());

    }

    else {
    //
      // double time_diff = (v1.current_time - v1.previous_time)*pow(10,-9);
      double time_diff = 0.1;

      // double v = sqrt(pow((v1.current_x - v1.prev_x),2)+ pow((v1.current_z - v1.prev_z),2))/time_diff;
      double v = sqrt(pow((v1.current_x - v1.prev_x),2)+ pow((v1.current_z - v1.prev_z),2))*10;

      double px = v1.current_x;
      double py = v1.current_z;
      double psi = v1.current_psi;
      double throttle_value = v1.current_throttle;
      double steer_value = v1.current_steering;

      ROS_INFO("px = [%f] py = [%f] psi = [%f] v = [%f] dt = [%f]", px, py, psi, v, time_diff);




      vector<double> ptsx, ptsy;


      size_t i = 0;

      // ROS_INFO(" trajectory size [%ld]", v1.traj_x.size());

      while(v1.traj_x[i] < px && v1.traj_x.size() > 0){
        i = i+1;
        // ROS_INFO("exec while loop [%i] th time",i);
      }

      for (; i < v1.traj_x.size();i++){
        ptsx.push_back(v1.traj_x[i]);
        ptsy.push_back(v1.traj_y[i]);
      }

      // ROS_INFO("ptsx_sizze = [%d]",ptsx.size());
      // i = 0;
      // for (; i < v1.traj_x.size();i++){
      //   ROS_INFO("i = [%d] ptsx[i] = [%f] ptsy[i] = [%f]",ptsx[i],ptsy[i]);
      // }

      // ROS_INFO(" trajectory size [%ld] current trajectory size = [%ld]", v1.traj_x.size(),ptsx.size());

      if (ptsx.size() < 4){
        v1.break_publisher();
        rate.sleep();

        ros::shutdown();
      }

      /*
      * TODO: Calculate steering angle and throttle using MPC.
      *
      * Both are in between [-1, 1].
      *
      */

      //Display the waypoints/reference line
      //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
      // the points in the simulator are connected by a Yellow line
      //vector<double> next_x_vals;
      //vector<double> next_y_vals;

      // ROS_INFO("ptsx_size = [%f] \n",ptsx.size());

      //transform the waypoints to cars frame
      Eigen::VectorXd x_waypoints(ptsx.size());
      Eigen::VectorXd y_waypoints(ptsy.size());
      //

      // ROS_INFO("ptsx_size = [%d]",ptsx.size());
      for (i =0; i<ptsx.size(); i++){
        double dx = ptsx[i] - px;
        double dy = ptsy[i] - py;
        x_waypoints[i] = dx * cos(-psi) - dy * sin(-psi);
        y_waypoints[i] = dx * sin(-psi) + dy * cos(-psi);

        //ROS_INFO("x_way = [%f] y_way = [%f] \n",x_waypoints[i],y_waypoints[i]);
        // next_x_vals.push_back(x_waypoints[i]);
        // next_y_vals.push_back(y_waypoints[i]);
      }
      Eigen::VectorXd coeffs = polyfit(x_waypoints, y_waypoints, 3);
      double cte = polyeval(coeffs, 0);
      double epsi = -atan(coeffs[1]);
      Eigen::VectorXd state(6);
      state.fill(0.0);
      // consider latency
      double latency = 0.1;
      const double Lf = 0.14;
      state[0] = v * latency;
      state[2] = v * latency * steer_value / Lf;
      state[3] = v + throttle_value * latency;
      state[4] = cte + v * sin(epsi) * latency;
      state[5] = epsi + v * latency * steer_value / Lf;

      ROS_INFO("cte = [%f] epsi = [%f]", state[4], state[5]);
      //compute the actuator values
      vector<double> result = mpc.Solve(state, coeffs);
      steer_value = result[0];
      steer_value =  (1-(steer_value/deg2rad(25)))/2;
      throttle_value = result[1];
      v1.publisher(throttle_value,steer_value);
    }

    //
    // }
    //


    v1.prev_x = v1.current_x;
    v1.prev_z = v1.current_z;
    v1.previous_time = v1.current_time;

    rate.sleep();
    ros::spinOnce();


    // init_time = v1.getTime().toNSec();
    // initial = v1.pose_();
    // double a=0.2, b=0.2;
    // v1.publisher(a,b);
    //
    // final_time = v1.getTime().toNSec();
    // final = v1.pose_();
    // double vel = (final - initial)/(final_time - init_time);
    // std::cout<<vel<<endl;

  }

    return 0;
}




          //
          // double px = v1.x;
          // double py = v1.z;
          // double psi = v1.pitch;



          // v1.publisher(result[0], result[1]);









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
