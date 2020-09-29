#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
using namespace std;
class TeleopJoy{
public:
  TeleopJoy();
    private:
      void callBack(const sensor_msgs::Joy::ConstPtr& joy);
      ros::NodeHandle n;
      ros::Publisher pub;
      ros::Subscriber sub;
      int i_velLinear, i_velAngular;
};

TeleopJoy::TeleopJoy(){
  n.param("axis_linear",i_velLinear,i_velLinear);
  n.param("axis_angular",i_velAngular,i_velAngular);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack,this);
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy){
  geometry_msgs::Twist vel;
  vel.linear.y = joy->axes[3];
  vel.linear.x = joy->axes[1];
  pub.publish(vel);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "teleopJoy");
  TeleopJoy teleop_turtle;
  ros::spin();
}