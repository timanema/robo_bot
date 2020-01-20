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

boolean stop = true;
int direction = 90;
float speed = 0.0;
int noInput = 0;
uint8_t motor1Forwards = 0;
uint8_t motor1Backwards = 0;
uint8_t motor2Forwards = 0;
uint8_t motor2Backwards = 0;

//ISR(TIMER4_OVF_vect) {
//  for(int i = 0; i < 255; i++) {
//    digitalWrite(PIN_MOTOR1_FWD, i < motor1Forwards ? HIGH : LOW);
//    digitalWrite(PIN_MOTOR1_REV, i < motor1Backwards ? HIGH : LOW);
//    digitalWrite(PIN_MOTOR2_FWD, i < motor2Forwards ? HIGH : LOW);
//    digitalWrite(PIN_MOTOR2_REV, i < motor2Backwards ? HIGH : LOW);
//  }
//}

void handle( const geometry_msgs::Twist& msg){
  direction = 180.0 * ((msg.angular.z + 1.0) / 2.0);
  speed = msg.linear.x;
  noInput = 0;
  stop = false;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle);

void setup()
{
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

//  cli();
//  TCCR4A = 0;
//  TCCR4B = 0;
//  
//  TIMSK4 = (1 << TOIE4);
//  TCCR4B |= (1 << CS10);
//  TCCR4B |= (1 << CS12);

  // Setup subscriber
  nh.initNode();
  nh.subscribe(sub);
//  sei();
}

void loop() {
  noInput += 1;
  //cli();

  nh.spinOnce();
  
  if (noInput >= 50) {
    stop = true;
  }
  
  digitalWrite(PIN_DIST_TRIG, LOW);
  digitalWrite(PIN_DIST_TRIG, HIGH);
  digitalWrite(PIN_DIST_TRIG, LOW);
  long duration = pulseIn(PIN_DIST_ECHO, HIGH);
  long distance = (duration / 2) / 29.1;
  //sei();

  digitalWrite(PIN_MOTOR1_EN, HIGH);
  digitalWrite(PIN_MOTOR2_EN, HIGH);

  if (!stop && (distance > 10 || distance <= 0)) {
    motor1Backwards = 0;
    motor2Backwards = 0;

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

    analogWrite(PIN_MOTOR1_FWD, int(left));//motor1Forwards = int(left);
    analogWrite(PIN_MOTOR2_FWD, int(right));//motor2Forwards = int(right);
    analogWrite(PIN_MOTOR1_REV, 0);//motor1Backwards = 0;
    analogWrite(PIN_MOTOR1_REV, 0);//motor2Backwards = 0;
    digitalWrite(PIN_LED, LOW);
  } else if(!stop && distance <= 5) {
    // Backwards!
    analogWrite(PIN_MOTOR1_FWD, 0);//motor1Forwards = 0;
    analogWrite(PIN_MOTOR2_FWD, 0);//motor2Forwards = 0;
    analogWrite(PIN_MOTOR1_REV, 255);//motor1Backwards = 255;
    analogWrite(PIN_MOTOR2_REV, 255);//motor2Backwards = 255;
    digitalWrite(PIN_LED, HIGH);
  } else {
    // Stop!
    digitalWrite(PIN_MOTOR1_EN, LOW);
    digitalWrite(PIN_MOTOR2_EN, LOW);
    analogWrite(PIN_MOTOR1_FWD, 0);//motor1Forwards = 0;
    analogWrite(PIN_MOTOR2_FWD, 0);//motor2Forwards = 0;
    analogWrite(PIN_MOTOR1_REV, 0);//motor1Backwards = 0;
    analogWrite(PIN_MOTOR2_REV, 0);//motor2Backwards = 0;
    digitalWrite(PIN_LED, LOW);
  }
}
