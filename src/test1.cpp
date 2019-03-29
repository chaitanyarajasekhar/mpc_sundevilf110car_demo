
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"


class test
{
public:
  ros::NodeHandle nh;
  ros::Subscriber pose_sub;
  // ros::Publisher speed_pub, throt_pub, steer_pub;
  geometry_msgs::PoseStamped pose;
  std_msgs::Float64 speed, steer, throt;

  ros::Publisher speed_pub=nh.advertise<std_msgs::Float64>("commands/motor/speed",1);
  ros::Publisher throt_pub =  nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle",1);
  ros::Publisher steer_pub=  nh.advertise<std_msgs::Float64>("commands/servo/position",1);;

test(ros::NodeHandle &n){
  sub();
  // pub();

}

void sub(){
  pose_sub = nh.subscribe("vrpn_client_node/f110car_2/pose",10,&test::poseCallback,this);
}

// void pub(){

// }


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose.header = msg->header;
  pose.pose = msg->pose;
  // pose.orientation = msg->orientation;
  // ROS_INFO("Pose y : [%f] ", msg->pose.position.y);
}

void publisher(){

  if(pose.pose.position.y <2){
    throt.data = 0.1;
    steer.data = 0.5;
    ROS_INFO("current y [%f]", pose.pose.position.y);
    throt_pub.publish(throt);
    steer_pub.publish(steer);
  }
  else{
    speed.data = 0;
    speed_pub.publish(speed);
  }
}

// private:
//



};


int main(int argc, char **argv){

  ros::init(argc,argv,"getpose");
  ros::NodeHandle n;
  // ros::Subscriber pose_sub = n.subscribe("vrpn_client_node/f110car_2/pose",10,poseCallback);
  test v1(n);

  ros::Rate rate(2);

  while(ros::ok()){
    v1.publisher();
    rate.sleep();
    ros::spinOnce();
  }

  return 0;;
}
