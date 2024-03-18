#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ESP32Encoder.h>

ESP32Encoder right_encoder;
// ESP32Encoder encoder2;
ros::NodeHandle nh;

std_msgs::Int32 right_encoder_msg;

ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);
// timer and flag for example, not needed for encoders
// unsigned long encoder2lastToggled;
// bool encoder2Paused = false;
#define m_pos 26
#define m_neg 25

double cmd_vel;
const double vel_threshold = 0.07; 
void right_motor_callback(const std_msgs::Float64& msg)
{
  cmd_vel = msg.data;
  if(abs(cmd_vel) > vel_threshold)
  {
    if(cmd_vel >= 0)
    {
      analogWrite(m_pos, map((1000 * cmd_vel), 0, 550, 0, 255));
      analogWrite(m_neg, 0);
    }
    else
    {
      analogWrite(m_neg, map((1000 * abs(cmd_vel)), 0, 550, 0, 255));
      analogWrite(m_pos, 0);
    }  
  }
  else
  {
    analogWrite(m_neg, 0);
    analogWrite(m_pos, 0);
  }
    
}

ros::Subscriber<std_msgs::Float64> right_motor_sub("/right_motor_cmd", &right_motor_callback);

void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 19 and 18 for the first encoder
	right_encoder.attachHalfQuad(14, 27);
	// use pin 17 and 16 for the second encoder
	// encoder2.attachHalfQuad(17, 16);
		
	// set starting count value after attaching
	right_encoder.setCount(0);

	// clear the encoder's raw count and set the tracked count to zero
	// encoder2.clearCount();
	// Serial.println("Encoder Start = " + String((int32_t)right_encoder.getCount()));
	// set the lastToggle
	// encoder2lastToggled = millis();
  pinMode(m_pos, OUTPUT);
  pinMode(m_neg, OUTPUT);

  nh.initNode();
  nh.advertise(right_encoder_pub);
  nh.subscribe(right_motor_sub);
}

void loop(){
	// Loop and read the count
  // static double t = 0.0;
  // t += 0.1;
  // analogWrite(m_pos, (uint8_t)(127 * (sin(t) + 1.0)));
  
	// Serial.print("CMD = " + String((uint8_t)(127 * (sin(t) + 1.0))));
  // Serial.println("    Encoder count = " + String((int32_t)encoder.getCount()));
  right_encoder_msg.data = (int32_t)right_encoder.getCount();
  right_encoder_pub.publish(&right_encoder_msg);
  nh.spinOnce();
	delay(50);
}

