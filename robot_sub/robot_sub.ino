#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define PIN_LED 13
#define PIN_DIST_TRIG 23  
#define PIN_DIST_ECHO 22
#define PIN_MOTOR1_EN 24
#define PIN_MOTOR1_FWD 6
#define PIN_MOTOR1_REV 7
#define PIN_MOTOR2_EN 25
#define PIN_MOTOR2_FWD 2
#define PIN_MOTOR2_REV 3

int direction = -42;
float speed = 0.0;
int noInput = 0;

void handle( const geometry_msgs::Twist& msg){
  direction = int(msg.angular.z);
  speed = msg.linear.x;
  noInput = 0;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle);

void setup()
{
  // Setup pins
  pinMode(PIN_DIST_TRIG, OUTPUT);
  pinMode(PIN_DIST_ECHO, INPUT);

  // Setup subscriber
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  noInput += 1;
  nh.spinOnce();
  delay(1);
  
  if (noInput >= 50) {
    direction = -42;
  }
  
  digitalWrite(PIN_DIST_TRIG, LOW);
  digitalWrite(PIN_DIST_TRIG, HIGH);
  digitalWrite(PIN_DIST_TRIG, LOW);
  long duration = pulseIn(PIN_DIST_ECHO, HIGH);
  long distance = (duration / 2) / 29.1;

  digitalWrite(PIN_MOTOR1_EN, HIGH);
  digitalWrite(PIN_MOTOR2_EN, HIGH);

  if (distance > 10 || distance <= 0){   
    analogWrite(PIN_MOTOR1_REV, 0);
    analogWrite(PIN_MOTOR2_REV, 0);

    float left = 0;
    float right = 0;
    
    if (direction < 0) {
      left = 0;
      right = 0;
    } else if (direction < 90) {
      left = 255;
      right = float(direction) / 90 * 255;
    } else if (direction > 90) {
      left = 255 - (float(direction) - 90) / 90 * 255;
      right = 255;
    } else {
      left = 255;
      right = 255;
    }
    
    left *= speed;
    right *= speed;
    
    analogWrite(PIN_MOTOR1_FWD, int(left));
    analogWrite(PIN_MOTOR2_FWD, int(right));
    
//  } else if(distance <= 5) {
 //   // Backwards!
//    analogWrite(PIN_MOTOR1_FWD, 0);
//    analogWrite(PIN_MOTOR1_REV, 255);
//    analogWrite(PIN_MOTOR2_FWD, 0);
//    analogWrite(PIN_MOTOR2_REV, 255);
  } else {
    // Stop!
    digitalWrite(PIN_MOTOR1_EN, LOW);
    digitalWrite(PIN_MOTOR2_EN, LOW);
    analogWrite(PIN_MOTOR1_FWD, 0);
    analogWrite(PIN_MOTOR1_REV, 0);
    analogWrite(PIN_MOTOR2_FWD, 0);
    analogWrite(PIN_MOTOR2_REV, 0);
  }
}
