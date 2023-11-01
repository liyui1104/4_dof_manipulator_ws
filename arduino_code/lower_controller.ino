#define MOTOR0_DIRPIN 2
#define MOTOR0_PULPIN 3
#define MOTOR1_DIRPIN 4
#define MOTOR1_PULPIN 5
#define MOTOR2_DIRPIN 6
#define MOTOR2_PULPIN 7
#define MOTOR3_DIRPIN 8
#define MOTOR3_PULPIN 9
#define PI 3.14159

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// SW1 --> on
// SW2 --> on
// SW3 --> off
// SW4 --> on
// SW5 --> off
// SW6 --> off
// SW7 --> off
// SW8 --> off
// The type of the stepper motor: 57CME23
// The type of the driver: CL57 H2-506
// The type of the MCU: Arduino Uno

ros::NodeHandle nh;
const int steps = 6400; // one circle --> 6400 pulses
const double loop_period = 0.1; // seconds

void messageCb(const std_msgs::Int16MultiArray& cmd_msg)
{
  for (int i = 0; i < 4; i++)
  {
    if (cmd_msg.data[i] >= 0)
    {
      run_stepper_motor(i,cmd_msg.data[i],loop_period,true);
    }
    else
    {
      run_stepper_motor(i,-cmd_msg.data[i],loop_period,false);
    }
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> subscriber("/lower_controller/command", &messageCb);

void setup()
{
  pinMode(MOTOR3_DIRPIN, OUTPUT);
  pinMode(MOTOR3_PULPIN, OUTPUT);
  Serial.begin(57600);

  nh.initNode();
  nh.subscribe(subscriber);
}

void run_stepper_motor(int sequence, int pulses, double seconds, bool dir)
{
  double rotate_speed = (9.0 * pulses) / (320 * seconds);  // = radians / PI / 2.0 / seconds * 60
  double pulse_period = 1000000 / (213.334 * rotate_speed);

  switch(sequence)
  {
    case 0:
    {
      if (dir) // CCW(anticlockwise)
      {
        digitalWrite(MOTOR0_DIRPIN,LOW);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR0_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR0_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      else // CW(clockwise)
      {
        digitalWrite(MOTOR0_DIRPIN,HIGH);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR0_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR0_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      break;
    }
    case 1:
    {
      if (dir) // CCW(anticlockwise)
      {
        digitalWrite(MOTOR1_DIRPIN,LOW);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR1_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR1_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      else // CW(clockwise)
      {
        digitalWrite(MOTOR1_DIRPIN,HIGH);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR1_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR1_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      break;
    }
    case 2:
    {
      if (dir) // CCW(anticlockwise)
      {
        digitalWrite(MOTOR2_DIRPIN,LOW);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR2_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR2_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      else // CW(clockwise)
      {
        digitalWrite(MOTOR2_DIRPIN,HIGH);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR2_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR2_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      break;
    }
    case 3:
    {
      if (dir) // CCW(anticlockwise)
      {
        digitalWrite(MOTOR3_DIRPIN,LOW);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR3_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR3_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      else // CW(clockwise)
      {
        digitalWrite(MOTOR3_DIRPIN,HIGH);
        for (int i = 0; i < pulses; i++)
        {
          digitalWrite(MOTOR3_PULPIN,HIGH);
          delayMicroseconds(pulse_period);
          digitalWrite(MOTOR3_PULPIN,LOW);
          delayMicroseconds(pulse_period);
        }
      }
      break;
    }
  }

}

void loop()
{
  nh.spinOnce();
  delay(10);
}