/*
 * Author: Timothy Baker (@tim-baker on GitHub)
 * Date: 13FEB14
 *
 * Demo heading-following code for the Pololu Zumo Robot
 *
 * This code will follow a heading using a
 * P-based algorithm.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 */

#include <Wire.h>
#include <LSM303.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>


ZumoBuzzer buzzer;
ZumoMotors motors;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);
int lastError = 0;

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int CNTR_SPEED = 0;
const int MAX_SPEED = 200;


void setup()
{
  // Play a little welcome song
  buzzer.play(">g32>>c32");
  
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-851, -294, -4096};
  compass.m_max = (LSM303::vector<int16_t>){+40, +929, -4096};
  
  motors.setSpeeds(0,0);
  
  // Wait for the user button to be pressed and released
  button.waitForButton();

  // Play music and wait for it to finish before we start driving.
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());
}

void loop()
{
  compass.read();
  
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){1, 0, 0});
  
  to use the +X axis as a reference.
  */
  float heading = compass.heading((LSM303::vector<int>){1, 0, 0});
  
  Serial.print("HEADING: ");
  Serial.print(heading);
  Serial.print("\t");
  
  // Get the position of the line.  Note that we *must* provide the "sensors"
  // argument to readLine() here, even though we are not interested in the
  // individual sensor readings

  // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  int error = heading - 180;
  
  Serial.print("ERROR: ");
  Serial.print(error);
  Serial.print("\n");

  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  int speedDifference = error;

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int m1Speed = CNTR_SPEED - speedDifference;
  int m2Speed = CNTR_SPEED + speedDifference;

  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  if (m1Speed < -MAX_SPEED)
    m1Speed = -MAX_SPEED;
  if (m2Speed < -MAX_SPEED)
    m2Speed = -MAX_SPEED;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;

  motors.setSpeeds(m1Speed, m2Speed);
}
