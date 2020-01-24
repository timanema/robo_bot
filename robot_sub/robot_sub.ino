#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "wiring_private.h"

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

void handle(const geometry_msgs::Twist& msg){
  direction = int(msg.angular.z);
  speed = msg.linear.x;
  noInput = 0;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle);

void setup() {
  // Setup pins
  pinMode(PIN_DIST_TRIG, OUTPUT);
  pinMode(PIN_DIST_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_MOTOR1_EN, OUTPUT);
  pinMode(PIN_MOTOR1_FWD, OUTPUT);
  pinMode(PIN_MOTOR1_REV, OUTPUT);
  pinMode(PIN_MOTOR2_EN, OUTPUT);
  pinMode(PIN_MOTOR2_FWD, OUTPUT);
  pinMode(PIN_MOTOR2_REV, OUTPUT);

  // Setup subscriber
  nh.initNode();
  nh.subscribe(sub);
}



void anal(uint8_t pin, int val) {
  switch(digitalPinToTimer(pin)) {
      // Right motor REV
      case TIMER3C:
        sbi(TCCR3A, COM3C1);
        OCR3C = val;
        break;

      // Left motor REV
      case TIMER4B:
        sbi(TCCR4A, COM4B1);
        OCR4B = val;
        break;
      
      // right motor FWR
      case TIMER3B:
        sbi(TCCR3A, COM3B1);
        OCR3B = val;
        break;

      // left motor FWR
      case TIMER4A:
        sbi(TCCR4A, COM4A1);
        cbi(TCCR4A, COM4A0);
        OCR4A = val;
        break;
    }
}

void loop() {
  noInput += 1;
  nh.spinOnce();
  
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

  if (distance > 20 || distance <= 0){
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

    // TODO: remove
    left = 100;
    right = 100;

    anal(PIN_MOTOR1_FWD, int(left));
    anal(PIN_MOTOR2_FWD, int(left));
    anal(PIN_MOTOR1_REV, 0);
    anal(PIN_MOTOR2_REV, 0);
    digitalWrite(PIN_LED, LOW);
  } else if(distance <= 10) {
    // Backwards!
    anal(PIN_MOTOR1_FWD, 0);
    anal(PIN_MOTOR2_FWD, 0);
    anal(PIN_MOTOR1_REV, 255);
    anal(PIN_MOTOR2_REV, 255);
    digitalWrite(PIN_LED, HIGH);
  } else {
    // Stop!
    digitalWrite(PIN_MOTOR1_EN, LOW);
    digitalWrite(PIN_MOTOR2_EN, LOW);
    anal(PIN_MOTOR1_FWD, 0);
    anal(PIN_MOTOR2_FWD, 0);
    anal(PIN_MOTOR1_REV, 0);
    anal(PIN_MOTOR2_REV, 0);
    digitalWrite(PIN_LED, LOW);
  }
}
