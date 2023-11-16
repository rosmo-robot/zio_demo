#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_RFD77402_Arduino_Library.h>

// Constants
#define MOTOR 0x40
#define SERVO 0x41
#define OLED_RESET 4
#define SERVOMIN 150
#define SERVOMAX 550

// Objects and variables
RFD77402 myDistance;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_PWMServoDriver pwm;
uint8_t servonum = 0;
uint16_t L_pulselen = 0;
uint16_t R_pulselen = 0;
unsigned int distance[5] = {0, 0, 0, 0, 0};

// Function declarations
void setMotorSpeed(uint8_t motorNum, uint16_t pulseLength);
void moveServo(uint16_t angle);
void measureDistance(uint8_t index, unsigned int *distanceArray);
void goAhead();
void stopMotors();
void turnRight();
void turnLeft();

void setup()
{
  Serial.begin(9600);

  // Initialize PWM driver for motor and servo
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  yield();

  // Initialize distance sensor
  if (!myDistance.begin())
  {
    Serial.println("Sensor failed to initialize. Check wiring.");
    while (1)
      ; // Freeze!
  }

  Serial.println("Sensor online!");
}

void loop()
{
  // Move the servo to the center position
  moveServo(90);
  delay(50);

  // Measure distance using the RFD77402 sensor
  myDistance.takeMeasurement();
  distance[2] = myDistance.getDistance();
  Serial.println(distance[0]);

  if (distance[2] > 300)
  {
    // Move forward
    goAhead();
    L_pulselen = R_pulselen = distance[2] + 200;
    setMotorSpeed(7, L_pulselen);
    setMotorSpeed(13, R_pulselen);

    while (distance[2] > 300)
    {
      myDistance.takeMeasurement();
      distance[2] = myDistance.getDistance();
      // Move the servo to the center position
      moveServo(90);
    }
  }
  else if (distance[2] <= 300)
  {
    // Stop motors
    stopMotors();
    // Move the servo to the extreme left position
    moveServo(0);
    delay(600);
    // Measure distance to the left
    measureDistance(0, distance);
    // Move the servo to the extreme right position
    moveServo(180);
    delay(600);
    // Measure distance to the right
    measureDistance(4, distance);
    // Move the servo to the center position
    moveServo(90);
    delay(600);

    if ((distance[0] > 300 && distance[4] > 300 && distance[0] > distance[4]) ||
        (distance[0] > 300 && distance[4] <= 300))
    {
      // Turn right
      turnRight();
      L_pulselen = R_pulselen = 600;
      setMotorSpeed(7, L_pulselen);
      setMotorSpeed(13, R_pulselen);

      while (distance[2] <= 300)
      {
        while (distance[3] <= 300)
        {
          // Move the servo to the right position
          moveServo(120);
          delay(100);
          // Measure distance to the right
          measureDistance(3, distance);
        }

        // Move the servo to the center position
        moveServo(90);
        delay(100);
        // Measure distance to the center
        measureDistance(2, distance);
      }
    }
    else if ((distance[0] > 300 && distance[4] > 300 && distance[0] < distance[4]) ||
             (distance[0] < 300 && distance[4] >= 300))
    {
      // Turn left
      turnLeft();
      L_pulselen = R_pulselen = 600;
      setMotorSpeed(7, L_pulselen);
      setMotorSpeed(13, R_pulselen);

      while (distance[2] < 400)
      {
        while (distance[1] <= 300)
        {
          // Move the servo to the left position
          moveServo(60);
          delay(100);
          // Measure distance to the left
          measureDistance(1, distance);
        }

        // Move the servo to the center position
        moveServo(90);
        delay(100);
        // Measure distance to the center
        measureDistance(2, distance);
      }
    }
  }
  else
  {
    // Stop motors
    stopMotors();
  }
}

// Function to set the motor speed using the Adafruit_PWMServoDriver
void setMotorSpeed(uint8_t motorNum, uint16_t pulseLength)
{
  pwm.setPWM(motorNum, 0, pulseLength);
}

// Function to move the servo to a specific angle
void moveServo(uint16_t angle)
{
  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulselen);
}

// Function to measure distance and store it in the distance array
void measureDistance(uint8_t index, unsigned int *distanceArray)
{
  myDistance.takeMeasurement();
  distanceArray[index] = myDistance.getDistance();
}

// Functions for different movements
void goAhead()
{
  // Implement forward movement logic
  // ...
}

void stopMotors()
{
  // Implement logic to stop motors
  // ...
}

void turnRight()
{
  // Implement right turn logic
  // ...
}

void turnLeft()
{
  // Implement left turn logic
  // ...
}
