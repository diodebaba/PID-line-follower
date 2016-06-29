#include <QTRSensors.h>
#define NUM_SENSORS   5    // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

//QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7},
QTRSensorsRC qtrrc((unsigned char[]) {7,6,5,4,3},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 


int leftp = 13;
int leftn = 12;
int leftpwm = 11; 

int rightp = 8;
int rightn = 9;
int rightpwm = 10;
int finalleftspeed;
int finalrightspeed;

int lastError = 0;

//so far kp ==0.08 is the best

//so far kd ==1.8 is the best
float KP = 0.08;
float KD = 1.6;

int M1 = 180;
int M2 = 135;

void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  pinMode(leftn, OUTPUT);
  pinMode(leftp, OUTPUT);
  pinMode(leftpwm, OUTPUT);
  pinMode(rightp, OUTPUT);
  pinMode(rightn, OUTPUT);
  pinMode(rightpwm, OUTPUT);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int sensorValues[NUM_SENSORS];
  int  position = qtrrc.readLine(sensorValues);
  int error = position - 2000;

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;
 
    
    finalleftspeed = constrain(m1Speed,0,180);
    finalrightspeed = constrain(m2Speed,0,135);
    
    Serial.print("m1Speed "); 
    Serial.print(finalleftspeed); // comment this line out if you are using raw values
    Serial.print(" ");
    Serial.print("m2Speed "); 
    Serial.print(finalrightspeed); // comment this line out if you are using raw values
    // The motor speed should not exceed the max PWM value
    
    if (m1Speed < 0)
    m1Speed = 0;
   if (m2Speed < 0)
    m2Speed = 0;
    Serial.println('\t');
    
  front();
  delay(250);
}

void front()
{
  
  analogWrite(leftpwm, finalleftspeed);
  digitalWrite(leftp, HIGH); 
  digitalWrite(leftn, LOW);

  analogWrite(rightpwm, finalrightspeed);
  digitalWrite(rightp, HIGH);
  digitalWrite(rightn, LOW);
  
}

