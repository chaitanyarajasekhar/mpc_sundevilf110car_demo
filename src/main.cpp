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

/*************************************************************************************/
// functions to convert from degrees to radians and vice versa

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


/***************** **********************************************************************/
// class to handle publisher and subscriber for vesc and motion capture
//--------------------------------------------------------------------------------------//

class f110car_vesc_mocap
{
public:
  ros::NodeHandle nh;
  ros::Subscriber pose_sub_car1, pose_sub_car2;

  geometry_msgs::PoseStamped pose_car1, pose_car2;
  geometry_msgs::Quaternion quat_car1, quat_car2;


  double roll_car1, pitch_car1 , yaw_car1;
  double roll_car2, pitch_car2, yaw_car2;

  double current_x, current_z, prev_x, prev_z, current_psi, current_time, previous_time;
  double current_x_car2, current_z_car2, prev_x_car2, prev_z_car2, current_psi_car2;

  double prev_velocity_car2, prev_acc_car2;


  double current_throttle, current_steering;

  vector<double> traj_x_car1, traj_y_car1;
  vector<double> traj_x_car2, traj_y_car2;

  std_msgs::Float64 speed_car1, steer_car1, throt_car1;
  std_msgs::Float64 speed_car2, steer_car2, throt_car2;

  ros::Time timenow = ros::Time::now() ;
  ros::Time time_var;



  ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("commands/motor/speed",1);
  ros::Publisher throt_pub =  nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle",1);
  ros::Publisher steer_pub =  nh.advertise<std_msgs::Float64>("commands/servo/position",1);;

  // ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("commands/motor/speed",1);
  // ros::Publisher throt_pub =  nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle",1);
  // ros::Publisher steer_pub =  nh.advertise<std_msgs::Float64>("commands/servo/position",1);

  f110car_vesc_mocap(ros::NodeHandle &n){
    sub();
    prev_velocity_car2 = 0; prev_acc_car2 = 0;
  }

  void sub()
  {
    pose_sub_car1 = nh.subscribe("vrpn_client_node/f110car_1/pose",10,&f110car_vesc_mocap::poseCallback_car1,this);
    pose_sub_car2 = nh.subscribe("vrpn_client_node/f110car_2/pose",10,&f110car_vesc_mocap::poseCallback_car2,this);

  }

  void poseCallback_car1(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    time_var = msg->header.stamp;
    pose_car1.header = msg->header;
    pose_car1.pose = msg->pose;
    // ROS_INFO("pose_x = [%f]", pose.pose.position.x);
    quat_car1 = msg->pose.orientation;
    tf::Quaternion quat_temp;
    tf::quaternionMsgToTF(msg->pose.orientation, quat_temp);
    tf::Matrix3x3(quat_temp).getRPY(roll_car1,pitch_car1,yaw_car1);


  }

  void poseCallback_car2(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    //time_var = msg->header.stamp;
    pose_car2.header = msg->header;
    pose_car2.pose = msg->pose;
    // ROS_INFO("pose_x = [%f]", pose.pose.position.x);
    quat_car2 = msg->pose.orientation;
    tf::Quaternion quat_temp;
    tf::quaternionMsgToTF(msg->pose.orientation, quat_temp);
    tf::Matrix3x3(quat_temp).getRPY(roll_car2,pitch_car2,yaw_car2);
  }

  void publisher(double &throttle, double &steering)
  {

    current_throttle = throttle;
    current_steering = steering;
    steer_car1.data =  (1-(steering/deg2rad(25)))/2;
    throt_car1.data = throttle;
    ROS_INFO("throttle = [%f], steering = [%f]", throttle, steering);
    throt_pub.publish(throt_car1);
    steer_pub.publish(steer_car1);
  }

  void break_publisher(){
    speed_car1.data = 0;
    speed_pub.publish(speed_car1);
  }

  // double pose_(){
  //   return pose_car1.pose.position.x;
  // }
  //
  // ros::Time getTime(){
  //   return ros::Time::now();
  // }
};





//
// Evaluate the polynomial with coefficients at x
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
  f110car_vesc_mocap car_pub_sub_node(n);

  // initializing a hardcoded trajectory
  vector<double> traj_x_init, traj_y_init;

  for (size_t j = 0; j < 100; j++){
    traj_x_init.push_back(-3.63+.029*j);
    traj_y_init.push_back(0.48);
    // ROS_INFO("traj_x,traj_y = [%f , %f]",traj_x_init[j],traj_y_init[j]);
  }

  car_pub_sub_node.traj_x_car1 = traj_x_init;
  car_pub_sub_node.traj_y_car1 = traj_y_init;

  ROS_INFO("traj_x_size = [%d]", car_pub_sub_node.traj_x_car1.size());

  bool initial_cond = true;
  int initial_empty_ros_spin = 10;

  //
  int r = 10;
  ros::Rate rate(r);

  rate.sleep();
  rate.sleep();
  rate.sleep();


  while(ros::ok()){


    car_pub_sub_node.current_x = round(car_pub_sub_node.pose_car1.pose.position.x);
    car_pub_sub_node.current_z = round(car_pub_sub_node.pose_car1.pose.position.z);
    car_pub_sub_node.current_psi = car_pub_sub_node.pitch_car1;
    car_pub_sub_node.current_time = car_pub_sub_node.time_var.nsec;

    car_pub_sub_node.current_x_car2 = round(car_pub_sub_node.pose_car2.pose.position.x);
    car_pub_sub_node.current_z_car2 = round(car_pub_sub_node.pose_car2.pose.position.z);
    car_pub_sub_node.current_psi_car2 = car_pub_sub_node.pitch_car2;
    // ROS_INFO("v1_c_x = [%f] v1_c_y = [%f]  ", v1.current_x, v1.current_z);
    // ROS_INFO(" trajectory size while loop starting [%ld]", v1.traj_x.size());


    // v1.current_time = v1.timenow.nsec;
    // ROS_INFO("time = [%f] ",v1.current_time);

    // initial_empty_ros_spin using this so that motion capture gets the value in v1.current_x taking 5 spins
    if (initial_cond == true || initial_empty_ros_spin > 0){
      initial_cond = false;
      initial_empty_ros_spin = initial_empty_ros_spin -1;
      double a =0.0, b = 0.5;
      car_pub_sub_node.publisher(a,b);
      // ROS_INFO("traj_x_size in if block= [%d]", v1.traj_x.size());

    }

    else {
    //
      // double time_diff = (v1.current_time - v1.previous_time)*pow(10,-9);
      double time_diff = 0.1;

      // double v = sqrt(pow((v1.current_x - v1.prev_x),2)+ pow((v1.current_z - v1.prev_z),2))/time_diff;
      double v = sqrt(pow((car_pub_sub_node.current_x - car_pub_sub_node.prev_x),2)+ pow((car_pub_sub_node.current_z - car_pub_sub_node.prev_z),2))*10;

      double px = car_pub_sub_node.current_x;
      double py = car_pub_sub_node.current_z;
      double psi = car_pub_sub_node.current_psi;
      double throttle_value = car_pub_sub_node.current_throttle;
      double steer_value = car_pub_sub_node.current_steering;

      double velocity_car2 = sqrt(pow((car_pub_sub_node.current_x_car2 - car_pub_sub_node.prev_x_car2),2)+ pow((car_pub_sub_node.current_z_car2 - car_pub_sub_node.prev_z_car2),2))*10;
      double acc_car2 = (velocity_car2 -  car_pub_sub_node.prev_velocity_car2) * 10;

      ROS_INFO("px = [%f] py = [%f] psi = [%f] v = [%f] dt = [%f]", px, py, psi, v, time_diff);
      ROS_INFO("px_car2 = [%f] py_car2 = [%f] psi_car2 = [%f] v = [%f] acc = [%f]", car_pub_sub_node.current_x_car2, car_pub_sub_node.current_z_car2, car_pub_sub_node.current_psi_car2, velocity_car2, acc_car2);




      vector<double> ptsx, ptsy;


      size_t i = 0;

      // selecting points infront of the car
      while(car_pub_sub_node.traj_x_car1[i] < px && car_pub_sub_node.traj_x_car1.size() > 0){
        i = i+1;
      }

      for (; i < car_pub_sub_node.traj_x_car1.size();i++){
        ptsx.push_back(car_pub_sub_node.traj_x_car1[i]);
        ptsy.push_back(car_pub_sub_node.traj_y_car1[i]);
      }

      // end the ros-spin loop if trajectory size is less than 4 because polyfit fnction need atleast size 3
      if (ptsx.size() < 4){
        car_pub_sub_node.break_publisher();
        rate.sleep();

        ros::shutdown();
      }

      /*
      * Calculate steering angle and throttle using MPC.
      *
      */


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

      Eigen::VectorXd state_other_car(7);
      state_other_car.fill(0.0);

      double psi_car2 = car_pub_sub_node.current_psi_car2;

      // state_other_car[0] = car_pub_sub_node.current_x_car2 + velocity_car2 * cos(psi_car2) *latency;
      // state_other_car[1] = car_pub_sub_node.current_z_car2 - velocity_car2 * sin(psi_car2) * latency;
      // state_other_car[2] = psi_car2;
      // state_other_car[3] = velocity_car2 + acc_car2 * latency;

      state_other_car[0] = -1.39 + velocity_car2 * cos(psi_car2) *latency;
      state_other_car[1] = 0.47 - velocity_car2 * sin(psi_car2) * latency;
      state_other_car[2] = 0;
      state_other_car[3] = velocity_car2 + acc_car2 * latency;

      // including the px py psi of car1
      state_other_car[4] = px;
      state_other_car[5] = py;
      state_other_car[6] = psi;

      // double dx = -1.48 - px;
      // double dy = 0.50 - py;
      // state_other_car[0] = dx * cos(-psi) - dy * sin(-psi);
      // state_other_car[1] = dx * sin(-psi) + dy * cos(-psi);

      vector<double> result = mpc.Solve(state, coeffs,state_other_car);

      steer_value = result[0];
      throttle_value = result[1];
      car_pub_sub_node.publisher(throttle_value,steer_value);


      car_pub_sub_node.prev_velocity_car2 = velocity_car2;
      //car_pub_sub_node.prev_acc_car2 = acc_car2;
    }



    car_pub_sub_node.prev_x = car_pub_sub_node.current_x;
    car_pub_sub_node.prev_z = car_pub_sub_node.current_z;
    car_pub_sub_node.previous_time = car_pub_sub_node.current_time;

    car_pub_sub_node.prev_x_car2 = car_pub_sub_node.current_x_car2;
    car_pub_sub_node.prev_z_car2 = car_pub_sub_node.current_z_car2;

    rate.sleep();
    ros::spinOnce();


  }

    return 0;
}
