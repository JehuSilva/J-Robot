#include <ros.h>
#include <std_msgs/String.h>
#include <Arduino.h>
#include <SPI.h>
//#include <WiFi.h>
#include "WiFiHardware.h"
 #include <geometry_msgs/Twist.h>

/***********************************/
/******    WiFi Settings   ******/
const char* ssid = "JJ";
const char* password = "Andri113_.";
/***********************************/
float x;
float y;
int dir11=14;
int dir12=27;
int dir21=26;
int dir22=25;
int en1=22;
int en2=23;

void leftMotor(int i){
  if(i==1){
      digitalWrite(dir11, LOW);
      digitalWrite(dir12, HIGH);
  }else{
      digitalWrite(dir11, HIGH);
      digitalWrite(dir12, LOW);
    }
}
void rightMotor(int i){
  if(i==1){
      digitalWrite(dir21, LOW);
      digitalWrite(dir22, HIGH);
  }else{
      digitalWrite(dir21, HIGH);
      digitalWrite(dir22, LOW);
    }
}
void moveMotors(float a,float b){


    if(a>0){
      leftMotor(1);
    }else{
      leftMotor(0);
    }

    if(b>0){
      rightMotor(1);
    }else{
      rightMotor(0);
    }

    a=constrain(a,0,1);
    b=constrain(b,0,1);
    ledcWrite(1, 255*a);
    ledcWrite(2, 255*b);
}
/************** ROS Definitions ***********************************/
//void chatterCallback(const std_msgs::String& msg) {
void chatterCallback(const geometry_msgs::Twist& vel) {
  //i = atoi(msg.data);
  x=vel.linear.x;
  y=vel.linear.y;
  moveMotors(x+y,x-y);    
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &chatterCallback);
ros::NodeHandle_<WiFiHardware> nh;
/*****************************************************************/

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(dir11,OUTPUT);
  pinMode(dir12,OUTPUT);
  pinMode(dir21,OUTPUT);
  pinMode(dir22,OUTPUT);
  ledcAttachPin(en1, 1);
  ledcAttachPin(en2, 2);
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcWrite(1, 100);
  ledcWrite(2, 100);
}

void loop() {
  nh.spinOnce();
  delay(500);
}
