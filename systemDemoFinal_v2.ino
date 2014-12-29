/*******************************************/  
/*******************************************/  
/**************** Libraries ****************/  
/*******************************************/  
/*******************************************/
#include <ArduinoUnit.h>
#include <TimerOne.h>
#include <Servo.h>



/*******************************************/  
/*******************************************/  
/***************** Servos ******************/  
/*******************************************/  
/*******************************************/
Servo servo1;
Servo servo2;



/*******************************************/  
/*******************************************/  
/****************** Pins *******************/  
/*******************************************/  
/*******************************************/

// System LED
const int LED_In1 = 52;
const int LED_Out1 =50;
const int LED_In2 = 48;
const int LED_Out2 =46;
const int ambientLEDPin_high = 42;
const int ambientLEDPin_low = 44;
const int systemLEDPin_high = 38;
const int systemLEDPin_low = 40;

// Servos
const int servo1Pin = 11;
const int servo2Pin = 12;

// Ultrasonic Rangefinders
const int ultrasonic1Pin = 4;
const int ultrasonic2Pin = 5;

// Light Ambient Sensor
const int ambientPower_high = A8;
const int ambientPower_low = A9;
const int ambientPin = A10;

// Accelerometer
const int accXPin = A0;
const int accYPin = A1;

// Color Sensors
const int color1Pin = 18;
const int color2Pin = 19;
const int color3Pin = 20;
const int color4Pin = 21;

const int colorFilter2Pin = 7;
const int colorFilter3Pin = 6;



/*******************************************/  
/*******************************************/
/*************** Constants *****************/  
/*******************************************/  
/*******************************************/  

// Servos
const int servo1Neutral = 1490;
const int servo2Neutral = 1495;
const int servoRespDelay = 10;
const int initBaudErr1_down = 100;
const int initBaudErr2_down = 80;
const int initBaudErr1_up = 380;
const int initBaudErr2_up = 390;
const int initBaudErr1_enter = 62;
const int initBaudErr2_enter = 70;
const int initBaudErr1_enter2 = 110;
const int initBaudErr2_enter2 = 110;
const int initBaudErr1_reenter = 130;
const int initBaudErr2_reenter = 100;
const int initBaudErr1_turn = 160;
const int initBaudErr2_turn = 160;

// Light Ambient Sensor
const int ambientThresh = 800;

// Time
const int numStep_down = 130;
const int numStep_up = 100;
const int numStep_back = 30;
const int numStep_turnLeft = 120;
const int numStep_turnRight = 100;
const int numStep_brake = 20;
const int numStep_right_Uturn_up = 33;
const int numStep_right_Uturn_down = 31;
// Accelerometer
const int bodyAngleThresh = 0.1;
const int accBias = 338;
const double downAngle = 87;
const double upAngle = 274.5;
const double rightAngle = 190;


// Ultrasonic Rangefinders
const double downCheckPoint[5] = {65.5, 50, 31, 11.57, 4.0};
const double upCheckPoint[5] = {4.0, 20.57, 40.5, 57, 67};

// Color Sensors
const int thred = 100;
const int thred_b = 200;
const int thred_c = 100;
const int thred1 = 300;
const int thred2 = 200;
const int delaytime = 5000;
const int grayThresh = 900;
const int boundThresh = 650;
const int inquiryOrder[100] = { 63,64,72,73,
                                45,46,54,55,
                                27,28,36,37,
                                 9,10,18,19,
                                 0, 1,81,82,
                                12,11, 3, 2,
                                30,29,21,20,
                                48,47,39,38,
                                66,65,57,56,
                                75,74,81,82,
                                67,68,76,77,
                                49,50,58,59,
                                31,32,40,41,
                                13,14,22,23,
                                 4, 5,81,82,
                                16,15, 7, 6,
                                34,33,25,24,
                                52,51,43,42,
                                70,69,61,60,
                                79,78,81,82,
                                81,71,82,80,
                                81,53,82,62,
                                81,35,82,44,
                                81,17,82,26,
                                81, 8,82,83,
                              };




/*******************************************/  
/*******************************************/
/*************** Variables *****************/  
/*******************************************/  
/*******************************************/

// Light Ambient Sensor
int ambientValue;
int switchState;

// Servos
int servo1Baud_init;
int servo2Baud_init;
int servo1Baud_adj;
int servo2Baud_adj;
double servoBaudAdd;

// Accelerometer
double bodyAngle;
double accXValue;
double accYValue;

// Ultrasonic Rangefinders
double val;
double distNearArray[5];
double distNear;
double distFarArray[5];
double distFar;

// Color Sensors
int isBound;

byte sensor1;
byte sensor2;
byte sensor3;
byte sensor4;

byte sensor1_c;
byte sensor2_c;
byte sensor3_c;
byte sensor4_c;

int grayScale1;
int grayScale2;
int grayScale3;
int grayScale4;

byte defectMapArray_b[84];
byte defectMapArray_c[84];
int visitingIdx = 0;

int g_count_1 = 0;
int g_count_2 = 0;
int g_count_3 = 0;
int g_count_4 = 0;

int g_array_1[3] = {0,0,0};
int g_array_2[3] = {0,0,0};
int g_array_3[3] = {0,0,0};
int g_array_4[3] = {0,0,0};

int ColorLib1[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int ColorLib2[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int ColorLib3[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int ColorLib4[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

int grayscale_lib_1[5] = {1550,0,0,200,950};
int grayscale_lib_2[5] = {1550,0,0,200,950};
int grayscale_lib_3[5] = {1350,0,0,200,800};
int grayscale_lib_4[5] = {1300,0,0,200,800};


int g_flag = 0;

int err_r;
int err_b;
int err_g;
int m;
int n;
int p;


/*******************************************/  
/*******************************************/
/*************** Functions *****************/  
/*******************************************/  
/*******************************************/  

// Light Ambient Switch to control the system ON/OFF
void ambientSwitch(void)
{
  
  brake();
  
  switchState = 1;
  digitalWrite(ambientPower_high, HIGH);
  digitalWrite(ambientPower_low, LOW);
  digitalWrite(ambientLEDPin_high, HIGH);
  digitalWrite(ambientLEDPin_low, LOW);
  digitalWrite(systemLEDPin_high, LOW);
  digitalWrite(systemLEDPin_low, LOW);
  
  while(switchState ==1)
  {
    grayScale1 = (g_array_1[0] + g_array_1[1] + g_array_1[2])/3;
    grayScale2 = (g_array_2[0] + g_array_2[1] + g_array_2[2])/3;
    grayScale3 = (g_array_3[0] + g_array_3[1] + g_array_3[2])/3;
    grayScale4 = (g_array_4[0] + g_array_4[1] + g_array_4[2])/3;
    Serial.print(grayScale1);
    Serial.print(' ');
    Serial.print(grayScale2);
    Serial.print(' ');
    Serial.print(grayScale3);
    Serial.print(' ');
    Serial.print(grayScale4);
    Serial.print('\n');
    
    delay(500);

    
    ambientValue = analogRead(ambientPin);
    //Serial.println(ambientValue);
    if(ambientValue >= ambientThresh)
    {
      switchState = 0;
      digitalWrite(ambientLEDPin_high, LOW);
      digitalWrite(ambientLEDPin_low, LOW);
      digitalWrite(systemLEDPin_high, HIGH);
      digitalWrite(systemLEDPin_low, LOW);
      return;
    }
  }
}

// Compute the body angle of the robot
void computeBodyAngle(void)
{  
  double bodyAngle_i;
  double bodyAngleArray[10];
  for(int i=0;i<10;i++)
  {
    accXValue = (double)analogRead(accXPin);
    accYValue = (double)analogRead(accYPin);
    double gx = accXValue - accBias;
    double gy = accYValue - accBias;
    if(gx<0)
    {
      bodyAngle_i = (atan(-gy/gx)+PI/2.0)*180/PI;
    }
    if(gx==0)
    {
      bodyAngle_i = 0;
    }
    if(gx>0)
    {
      bodyAngle_i = (atan(-gy/gx)+1.5*PI )*180/PI;
    }
    bodyAngleArray[i] = bodyAngle_i;
    delay(2);  
  }
  bodyAngle = mid(bodyAngleArray, 10);
  //Serial.println(bodyAngle);
}

// Motion brake
void brake(void)
{
  for(int i=1;i<=numStep_brake;i++)
  {
    servo1.writeMicroseconds(servo1Neutral);
    servo2.writeMicroseconds(servo2Neutral);
    delay(servoRespDelay);  
  }
}

// Start the motion as facing downward
void startMove_down(void)
{
  servo1Baud_init = servo1Neutral + initBaudErr1_down;
  servo2Baud_init = servo2Neutral - initBaudErr2_down;
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

// Start the motion as facing upward
void startMove_up(void)
{
  servo1Baud_init = servo1Neutral + initBaudErr1_up;
  servo2Baud_init = servo2Neutral - initBaudErr2_up;
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

// Start the backward motion
void startMove_back(void)
{
  servo1Baud_init = servo1Neutral - initBaudErr1_up;
  servo2Baud_init = servo2Neutral + initBaudErr2_up; 
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

// Continue maving back as facing downward
void moveBackward_down(int bodyAngle)
{
  if(bodyAngle>downAngle && abs(bodyAngle - downAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - downAngle);
    double alpha = 10;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init + servoBaudAdd;
    servo2Baud_adj = servo2Baud_init + servoBaudAdd;
  }
  if(bodyAngle<downAngle && abs(bodyAngle - downAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - downAngle);
    double alpha = 10;
    servoBaudAdd = alpha*diff;

    servo1Baud_adj = servo1Baud_init - servoBaudAdd;
    servo2Baud_adj = servo2Baud_init - servoBaudAdd;
  }
  
  servo1.writeMicroseconds(servo1Baud_adj);
  servo2.writeMicroseconds(servo2Baud_adj);
  delay(servoRespDelay);
}

// Continue moving backward as facing upward
void moveBackward_up(int bodyAngle)
{
  if(bodyAngle>downAngle && abs(bodyAngle - upAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - upAngle);
    double alpha = 4;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init + servoBaudAdd;
    servo2Baud_adj = servo2Baud_init + servoBaudAdd;
  }
  if(bodyAngle<downAngle && abs(bodyAngle - downAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - downAngle);
    double alpha = 4;
    servoBaudAdd = alpha*diff;

    servo1Baud_adj = servo1Baud_init - servoBaudAdd;
    servo2Baud_adj = servo2Baud_init - servoBaudAdd;
  }
  
  servo1.writeMicroseconds(servo1Baud_adj);
  servo2.writeMicroseconds(servo2Baud_adj);
  delay(servoRespDelay);
}

// Move downward
void moveDownward(int bodyAngle)
{
  if(bodyAngle>downAngle && abs(bodyAngle - downAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - downAngle);
    double alpha = 3;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init + servoBaudAdd;
    servo2Baud_adj = servo2Baud_init + servoBaudAdd;
  }
  if(bodyAngle<downAngle && abs(bodyAngle - downAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - downAngle);
    double alpha = 3;
    servoBaudAdd = alpha*diff;

    servo1Baud_adj = servo1Baud_init - servoBaudAdd;
    servo2Baud_adj = servo2Baud_init - servoBaudAdd;
  }
  
  servo1.writeMicroseconds(servo1Baud_adj);
  servo2.writeMicroseconds(servo2Baud_adj);
  delay(servoRespDelay);
}

// Move upward
void moveUpward(int bodyAngle)
{
  if(bodyAngle>upAngle && abs(bodyAngle - upAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - upAngle);
    double alpha = 20;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init + servoBaudAdd;
    servo2Baud_adj = servo2Baud_init + servoBaudAdd;
  }
  if(bodyAngle<upAngle && abs(bodyAngle - upAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - upAngle);
    double alpha = 20;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init - servoBaudAdd;
    servo2Baud_adj = servo2Baud_init - servoBaudAdd;
  }
  
  servo1.writeMicroseconds(servo1Baud_adj);
  servo2.writeMicroseconds(servo2Baud_adj);
  delay(servoRespDelay);
}

// Move right
void moveRight(int bodyAngle)
{
  if(bodyAngle>rightAngle && abs(bodyAngle - rightAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - rightAngle);
    double alpha = 15;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init + servoBaudAdd;
    servo2Baud_adj = servo2Baud_init + servoBaudAdd;
  }
  if(bodyAngle<rightAngle && abs(bodyAngle - rightAngle)>bodyAngleThresh)
  {
    double diff = abs(bodyAngle - rightAngle);
    double alpha = 10;
    servoBaudAdd = alpha*diff;
    servo1Baud_adj = servo1Baud_init - servoBaudAdd;
    servo2Baud_adj = servo2Baud_init - servoBaudAdd;
  }
  
  servo1.writeMicroseconds(servo1Baud_adj);
  servo2.writeMicroseconds(servo2Baud_adj);
  delay(servoRespDelay);
}

// Move right in enter procedure
void moveRight_enter(void)
{
  servo1Baud_init = servo1Neutral + initBaudErr1_enter;
  servo2Baud_init = servo2Neutral - initBaudErr2_enter;
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

void moveRight_enter2(void)
{
  servo1Baud_init = servo1Neutral + initBaudErr1_enter2;
  servo2Baud_init = servo2Neutral - initBaudErr2_enter2;
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

// Move left in re-enter procedure
void moveLeft_reenter(void)
{
  servo1Baud_init = servo1Neutral - initBaudErr1_reenter;
  servo2Baud_init = servo2Neutral + initBaudErr2_reenter;
  servo1.writeMicroseconds(servo1Baud_init);
  servo2.writeMicroseconds(servo2Baud_init);
  delay(servoRespDelay);
}

// Turn left
void turnLeft(void)
{
  servo1Baud_init = servo1Neutral - initBaudErr1_turn;
  servo2Baud_init = servo2Neutral - initBaudErr2_turn;

  for(int i=0;i<numStep_turnLeft;i++)
  {
    servo1.writeMicroseconds(servo1Baud_init);
    servo2.writeMicroseconds(servo2Baud_init);
    delay(servoRespDelay);
  }
}

// Turn right
void turnRight(void)
{
  servo1Baud_init = servo1Neutral + initBaudErr1_turn;
  servo2Baud_init = servo2Neutral + initBaudErr2_turn;

  for(int i=0;i<numStep_turnRight;i++)
  {
    servo1.writeMicroseconds(servo1Baud_init);
    servo2.writeMicroseconds(servo2Baud_init);
    delay(servoRespDelay);
  }
}

// Compute the median number of an array
double mid(double *a, int N)
{
  int i,j;
  double t;
  double middle;
  for(i=0;i<N;i++)
  {
    for(j=i+1;j<N;j++)
    {
      if(a[j]<a[i])
      {
        t=a[i];
        a[i]=a[j];
        a[j]=t;
       }
      }
   }
  for(i=0;i<N;i++)
  {
    if(N%2==1)
    {
      middle=a[N/2];
    }
    else
    {
      middle=(a[N/2]+a[N/2-1])/2.0;
    }
  }
  return middle;
}

// Get the value from Ultrasonic Rangefinder #!
void getDistNear(void)
{ 
  for(int i=0;i<5;i++)
  {
    pinMode(ultrasonic1Pin, OUTPUT);
    digitalWrite(ultrasonic1Pin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic1Pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(ultrasonic1Pin, LOW);
  
    pinMode(ultrasonic1Pin, INPUT);
  
    val = pulseIn(ultrasonic1Pin, HIGH);
    distNearArray[i] = (double)val/29.0/2.0;
    delay(5);
  }
  distNear = mid(distNearArray, 5);
}

// Get the value from Ultrasonic Rangefinder #2
void getDistFar(void)
{
  
  for(int i=0;i<5;i++)
  {
    pinMode(ultrasonic2Pin, OUTPUT);
    digitalWrite(ultrasonic2Pin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic2Pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(ultrasonic2Pin, LOW);
  
    pinMode(ultrasonic2Pin, INPUT);
  
    val = pulseIn(ultrasonic2Pin, HIGH);
    distFarArray[i] = (double)val/29.0/2.0;
    delay(5);
  }
  distFar = mid(distFarArray, 5);
}

// Adjust trait of the robot
void traitAdjust(int targetAngle)
{
  int prop = 8;
  int baudErr;
  int numAdjust = 0;
  computeBodyAngle();
  while(abs(bodyAngle-targetAngle)>=bodyAngleThresh)
  {
    if(bodyAngle>targetAngle)
    {
      baudErr = prop*abs(bodyAngle-targetAngle);
      servo1Baud_init = servo1Neutral + baudErr;
      servo2Baud_init = servo2Neutral + baudErr;
      servo1.writeMicroseconds(servo1Baud_init);
      servo2.writeMicroseconds(servo2Baud_init);
      delay(50);
    }
    if(bodyAngle<targetAngle)
    {
      baudErr = prop*abs(bodyAngle-targetAngle);
      servo1Baud_init = servo1Neutral - baudErr;
      servo2Baud_init = servo2Neutral - baudErr;
      servo1.writeMicroseconds(servo1Baud_init);
      servo2.writeMicroseconds(servo2Baud_init);
      delay(50);
    }
    delay(servoRespDelay);
    computeBodyAngle();
    numAdjust = numAdjust + 1;
    if(numAdjust>5)
    {
      break;
    }
  }
}

// Color Sensing
///////////////////////////////////////
void TSC_Init()
{
  pinMode(colorFilter2Pin, OUTPUT);
  pinMode(colorFilter3Pin, OUTPUT);
  pinMode(color1Pin, INPUT);
  pinMode(color2Pin, INPUT);
  pinMode(color3Pin, INPUT);
  pinMode(color4Pin, INPUT);
}

///////////////////////////////////////
void TSC_FilterColor(int level1, int level2)
{
  if(level1 != 0)
  {
    level1 = HIGH;
  }
  if(level2 != 0)
  {
    level2 = HIGH;
  }
  digitalWrite(colorFilter2Pin, level1);
  digitalWrite(colorFilter3Pin, level2);
}

///////////////////////////////////////
void TSC_WB(int level0, int level1)
{
  g_count_1 = 0;
  g_count_2 = 0;
  g_count_3 = 0;
  g_count_4 = 0;
  g_flag ++;
  TSC_FilterColor(level0, level1);
  Timer1.setPeriod(500000);
}

///////////////////////////////////////
void TSC_Count_1(void)
{
  g_count_1 ++ ;
}

///////////////////////////////////////
void TSC_Count_2(void)
{
  g_count_2 ++ ;
}

///////////////////////////////////////
void TSC_Count_3(void)
{
  g_count_3 ++ ;
}

///////////////////////////////////////
void TSC_Count_4(void)
{
  g_count_4 ++ ;
}

///////////////////////////////////////
void TSC_Callback()
{
  switch(g_flag)
  {
    case 0:
    {
      TSC_WB(LOW, LOW);
      break;
    }
    case 1:
    {         
      g_array_1[0] = g_count_1;
      g_array_2[0] = g_count_2;
      g_array_3[0] = g_count_3;
      g_array_4[0] = g_count_4;
         
      TSC_WB(HIGH, HIGH);
      break;
    }
    case 2:
    {    
      g_array_1[1] = g_count_1;
      g_array_2[1] = g_count_2;
      g_array_3[1] = g_count_3;
      g_array_4[1] = g_count_4;
      TSC_WB(LOW, HIGH);
      break;
    }
    case 3:
    {

      g_array_1[2] = g_count_1;
      g_array_2[2] = g_count_2;
      g_array_3[2] = g_count_3;
      g_array_4[2] = g_count_4;
      TSC_WB(HIGH, LOW);
      g_flag = 0;
      break;
    }
    default:
    {
      g_count_1 = 0;
      g_count_2 = 0;
      g_count_3 = 0;
      g_count_4 = 0;
      break;
    }
  }
}

///////////////////////////////////////
// Get color library
void getColorLibrary(void)
{
  Serial.println("Input Yellow");
  digitalWrite(LED_Out1, HIGH);
  
  delay(delaytime);

  digitalWrite(LED_Out1, LOW);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  
  for (int i=0; i<10; i++)
  {
    for (int j=0; j<3; j++)
    {
      ColorLib1[0][j] = g_array_1[j];
      ColorLib2[0][j] = g_array_2[j];
      ColorLib3[0][j] = g_array_3[j];
      ColorLib4[0][j] = g_array_4[j];
      delay(5);
    }
  }
  
  Serial.println("*********** Finished! ************");
 
  Serial.println("Yellow #1:");
  for (int j=0; j<3;j++)
  {
    Serial.println(ColorLib1[0][j]);
  }
  
  Serial.println("Yellow #2:"); 
  for (int j = 0; j<3;j++)
  {
    Serial.println(ColorLib2[0][j]);
  }
   
  Serial.println("Input PINK");
  digitalWrite(LED_Out1, HIGH);
  delay(delaytime);
  
  digitalWrite(LED_Out1, LOW);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  
  for (int i=0; i<10; i++)
  {
    for (int j=0; j<3; j++)
    {
      ColorLib1[1][j] = g_array_1[j];
      ColorLib2[1][j] = g_array_2[j];
      ColorLib3[1][j] = g_array_3[j];
      ColorLib4[1][j] = g_array_4[j];
      delay(5);
    }
  }
  
  Serial.println("*********** Finished! ************");
  
  Serial.println("PINK #1:");
  for (int j=0; j<3;j++)
  {
    Serial.println(ColorLib1[1][j]);
  }
  
  Serial.println("Pink #2:"); 
  for (int j = 0; j<3;j++)
  {
    Serial.println(ColorLib2[1][j]);
  }
  
  Serial.println("Input LIGHTBLUE");
  digitalWrite(LED_Out1, HIGH);
  delay(delaytime);
  
  digitalWrite(LED_Out1, LOW);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  
  for (int i=0; i<10; i++)
  {
    for (int j=0; j<3; j++)
    {
      ColorLib1[2][j] = g_array_1[j];
      ColorLib2[2][j] = g_array_2[j];
      ColorLib3[2][j] = g_array_3[j];
      ColorLib4[2][j] = g_array_4[j];
      delay(5);
    }
  }
  
  Serial.println("***********Finished!************");
  Serial.println("LIGHTBULE #1:"); 
  for (int j=0; j<3; j++)
  {
    Serial.println(ColorLib1[2][j]);
  }
  
  Serial.println("Lightblue #2:"); 
  for (int j = 0; j<3;j++)
  {
    Serial.println(ColorLib2[2][j]);
  }
  
  Serial.println("Input BLACK");
  digitalWrite(LED_Out1, HIGH);
  delay(delaytime);
  
  digitalWrite(LED_Out1, LOW);
 
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  
  for (int i=0; i<10; i++)
  {
    for (int j=0; j<3; j++)
    {
      ColorLib1[3][j] = g_array_1[j];
      ColorLib2[3][j] = g_array_2[j];
      ColorLib3[3][j] = g_array_3[j];
      ColorLib4[3][j] = g_array_4[j];
      delay(5);
    }
  }
  
  Serial.println("***********Finished!************");
  Serial.println("Black #1:");
  for (int j=0; j<3; j++)
  {
    Serial.println(ColorLib1[3][j]);
  }
  
  Serial.println("black #2:"); 
  for (int j = 0; j<3;j++)
  {
    Serial.println(ColorLib2[3][j]);
  }
  
  
  Serial.println("Input Gray");
  digitalWrite(LED_Out1, HIGH);
  delay(delaytime);
  
  digitalWrite(LED_Out1, LOW);
 
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  digitalWrite(LED_Out2, HIGH);
  delay(500);
  digitalWrite(LED_Out2, LOW);
  delay(500);
  
  for (int i=0; i<10; i++)
  {
    for (int j=0; j<3; j++)
    {
      ColorLib1[4][j] = g_array_1[j];
      ColorLib2[4][j] = g_array_2[j];
      ColorLib3[4][j] = g_array_3[j];
      ColorLib4[4][j] = g_array_4[j];
      delay(5);
    }
  }
  
  Serial.println("***********Finished!************");
  Serial.println("Gray #1:");
  for (int j=0; j<3; j++)
  {
    Serial.println(ColorLib1[4][j]);
  }
  Serial.println("gray #2:"); 
  for (int j = 0; j<3;j++)
  {
    Serial.println(ColorLib2[4][j]);
  }
  
  //build grayscale library
  Serial.println("Building up grayscale library...");
  for (int j=0; j<5; j++)
  {
    grayscale_lib_1[j] = (ColorLib1[j][0] + ColorLib1[j][1] + ColorLib1[j][2])/3;
    Serial.print(grayscale_lib_1[j]);
    Serial.print(' ');
    grayscale_lib_2[j] = (ColorLib2[j][0] + ColorLib2[j][1] + ColorLib2[j][2])/3;
    Serial.print(grayscale_lib_2[j]);
    Serial.print(' ');
    grayscale_lib_3[j] = (ColorLib3[j][0] + ColorLib3[j][1] + ColorLib3[j][2])/3;
    Serial.print(grayscale_lib_3[j]);
    Serial.print(' ');
    grayscale_lib_4[j] = (ColorLib4[j][0] + ColorLib4[j][1] + ColorLib4[j][2])/3;
    Serial.print(grayscale_lib_4[j]);
    Serial.print(' ');
    Serial.print('\n');
  }
  Serial.println("*********Fisnished building***********");
  Serial.println("**********Will START in 5 seconds**********");
  delay (5000);

}

// Check defects in baseline
byte checkDefect1_b(void)
{
  for (int i=0;i<5;i++)
  {
    delay(50);
    grayScale1 = (g_array_1[0] + g_array_1[1] + g_array_1[2])/3;
  }
  Serial.print(g_array_1[0]);
  Serial.print(' ');
  Serial.print(g_array_1[1]);
  Serial.print(' ');
  Serial.print(g_array_1[2]);
  Serial.print(' ');
  Serial.print(grayScale1);
  Serial.print(' ');
  int err_y = abs(grayScale1-grayscale_lib_1[0]);
  int err_g = abs(grayScale1-grayscale_lib_1[4]);
  int center = (grayscale_lib_1[0] + grayscale_lib_1[4])/2;
  if(grayScale1>center)
  {
    return 1;
  }
  else
  {
    return 0;
  }

}

byte checkDefect2_b(void)
{
  for (int i=0;i<5;i++)
  {
    delay(50);
    grayScale2 = (g_array_2[0] + g_array_2[1] + g_array_2[2])/3;
  }
  Serial.print(grayScale2);
  Serial.print(' ');
  int err_y = abs(grayScale2-grayscale_lib_2[0]);
  int err_g = abs(grayScale2-grayscale_lib_2[4]);
  int center = (grayscale_lib_2[0] + grayscale_lib_2[4])/2;
  if(grayScale2>center)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

byte checkDefect3_b(void)
{
  for (int i=0;i<5;i++)
  {
    delay(50);
    grayScale3 = (g_array_3[0] + g_array_3[1] + g_array_3[2])/3;
  }
  Serial.print(grayScale3);
  Serial.print(' ');
  int err_y = abs(grayScale3-grayscale_lib_3[0]);
  int err_g = abs(grayScale3-grayscale_lib_3[4]);
  int center = (grayscale_lib_3[0] + grayscale_lib_3[4])/2;
  if(grayScale3>center)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

byte checkDefect4_b(void)
{
  for (int i=0;i<5;i++)
  {
    delay(50);
    grayScale4 = (g_array_4[0] + g_array_4[1] + g_array_4[2])/3;
  }
  Serial.print(grayScale4);
  Serial.print(' ');
  int err_y = abs(grayScale4-grayscale_lib_4[0]);
  int err_g = abs(grayScale4-grayscale_lib_4[4]);
  int center = (grayscale_lib_4[0] + grayscale_lib_4[4])/2;
  if(grayScale4>center)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


int error_y;
int error_k;
int error_g;
int error_w;

int err_gr_r = 0;
int err_gr_b = 0;
int err_gr_g = 0;
  
int err_r_r = 0;
int err_r_g = 0;
int err_r_b = 0;
  
int err_g_r = 0;
int err_g_g = 0;
int err_g_b = 0;

// Check defects in coolness factors
int checkDefect1_c(void)
{
  grayScale1 = (g_array_1[0] + g_array_1[1] + g_array_1[2])/3;
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_y = abs(grayScale1 - grayscale_lib_1[0]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_k = abs(grayScale1 - grayscale_lib_1[3]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_g = abs(grayScale1 - grayscale_lib_1[4]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_r_r = abs(g_array_1[0] - ColorLib1[1][0]);
    err_r_g = abs(g_array_1[1] - ColorLib1[1][1]);
    err_r_b = abs(g_array_1[2] - ColorLib1[1][2]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_g_r = abs(g_array_1[0] - ColorLib1[2][0]);
    err_g_g = abs(g_array_1[1] - ColorLib1[2][1]);
    err_g_b = abs(g_array_1[2] - ColorLib1[2][2]);
  }
 
  for (int i = 0 ; i < 10; i++ )
  {
    err_gr_r = abs(g_array_1[0] - ColorLib1[4][0]);
    err_gr_b = abs(g_array_1[1] - ColorLib1[4][1]);
    err_gr_g = abs(g_array_1[2] - ColorLib1[4][2]);
  } 
  
  int center = (grayscale_lib_1[0] + grayscale_lib_1[4])/2;
  if(grayScale1>center)
  {
    return 1;
  }
  
  
  
  else if ( error_k <= thred1  )
  {
    //Serial.println("********* Black ***********");
    return 0;
  }
  
  else if (err_r_r <= 120 && err_r_g <= 100 && err_r_b <= 100)
  {
    //Serial.println("********* Pink ***********");
    return 4;
  }
  
  else if ( error_g <= thred1 )
  {
     if ( err_g_r <= thred2 && err_g_g <= thred2 && err_g_b <= thred2)
    {
      //Serial.println("********* Lightblue ************");
      return 3;
    }
    else if ( err_gr_r <= thred2 && err_gr_b <= thred2 && err_gr_g <= thred2 )
    {
      //Serial.println("********* Gray ************");
      return 0;
    }
  }
  else
  {
    return 2;
  }
  
}




int checkDefect2_c(void)
{
  grayScale2 = (g_array_2[0] + g_array_2[1] + g_array_2[2])/3;
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_y = abs(grayScale2 - grayscale_lib_2[0]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_k = abs(grayScale2 - grayscale_lib_2[3]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_g = abs(grayScale2 - grayscale_lib_2[4]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_r_r = abs(g_array_2[0] - ColorLib2[1][0]);
    err_r_g = abs(g_array_2[1] - ColorLib2[1][1]);
    err_r_b = abs(g_array_2[2] - ColorLib2[1][2]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_g_r = abs(g_array_2[0] - ColorLib2[2][0]);
    err_g_g = abs(g_array_2[1] - ColorLib2[2][1]);
    err_g_b = abs(g_array_2[2] - ColorLib2[2][2]);
  }
 
  for (int i = 0 ; i < 10; i++ )
  {
    err_gr_r = abs(g_array_2[0] - ColorLib2[4][0]);
    err_gr_b = abs(g_array_2[1] - ColorLib2[4][1]);
    err_gr_g = abs(g_array_2[2] - ColorLib2[4][2]);
  }
  
  int center = (grayscale_lib_2[0] + grayscale_lib_2[4])/2;
  if(grayScale2>center)
  {
    return 1;
  }
  
  else if ( error_k <= thred1 && err_g_g > thred2 && err_r_r > thred2 && error_k < error_g)
  {
    //Serial.println("********* Black ***********");
    return 0;
  }

  else if (err_r_r <= 100 && err_r_g <= 100 && err_r_b <= 100)
  {
    //Serial.println("********* Pink ***********");
    return 4;
  }
  
  else if ( error_g <= thred1 )
  {
    if ( err_g_r <= thred2 && err_g_g <= thred2 && err_g_b <= thred2)
    {
      //Serial.println("********* Lightblue ************");
      return 3;
    }
    else if ( err_gr_r <= thred2 && err_gr_b <= thred2 && err_gr_g <= thred2 )
    {
      //Serial.println("********* Gray ************");
      return 0;
    }
  }
  
  else
  {
    return 2;
  }
}




int checkDefect3_c(void)
{
  grayScale3 = (g_array_3[0] + g_array_3[1] + g_array_3[2])/3;
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_y = abs(grayScale3 - grayscale_lib_3[0]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_k = abs(grayScale3 - grayscale_lib_3[3]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_g = abs(grayScale3 - grayscale_lib_3[4]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_r_r = abs(g_array_3[0] - ColorLib3[1][0]);
    err_r_g = abs(g_array_3[1] - ColorLib3[1][1]);
    err_r_b = abs(g_array_3[2] - ColorLib3[1][2]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_g_r = abs(g_array_3[0] - ColorLib3[2][0]);
    err_g_g = abs(g_array_3[1] - ColorLib3[2][1]);
    err_g_b = abs(g_array_3[2] - ColorLib3[2][2]);
  }
 
  for (int i = 0 ; i < 10; i++ )
  {
    err_gr_r = abs(g_array_3[0] - ColorLib3[4][0]);
    err_gr_b = abs(g_array_3[1] - ColorLib3[4][1]);
    err_gr_g = abs(g_array_3[2] - ColorLib3[4][2]);
  }
  
  int center = (grayscale_lib_3[0] + grayscale_lib_3[4])/2;
  if(grayScale3>center)
  {
    return 1;
  }
  
  
  else if ( error_k <= thred1 && err_g_g > thred2 && err_r_r > thred2 && error_k < error_g)
  {
    //Serial.println("********* Black ***********");
    return 0;
  }

  else if ( err_r_r <= 100 && err_r_g <= 100 && err_r_b <= 100 )
    {
      //Serial.println("********* Pink ************");
      return 4;
    }
  else if ( error_g <= thred1 )
  {
     if ( err_g_r <= thred2 && err_g_g <= thred2 && err_g_b <= thred2)
    {
      //Serial.println("********* Lightblue ************");
      return 3;
    }
    else if ( err_gr_r <= thred2 && err_gr_b <= thred2 && err_gr_g <= thred2 )
    {
      //Serial.println("********* Gray ************");
      return 0;
    }
  }
  else
  {
    return 2;
  }
  
}




int checkDefect4_c(void)
{
  grayScale4 = (g_array_4[0] + g_array_4[1] + g_array_4[2])/3;
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_y = abs(grayScale4 - grayscale_lib_4[0]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_k = abs(grayScale4 - grayscale_lib_4[3]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    error_g = abs(grayScale4 - grayscale_lib_4[4]);
  }

  
  for (int i = 0 ; i < 10; i++ )
  {
    err_r_r = abs(g_array_4[0] - ColorLib4[1][0]);
    err_r_g = abs(g_array_4[1] - ColorLib4[1][1]);
    err_r_b = abs(g_array_4[2] - ColorLib4[1][2]);
  }
  
  for (int i = 0 ; i < 10; i++ )
  {
    err_g_r = abs(g_array_4[0] - ColorLib4[2][0]);
    err_g_g = abs(g_array_4[1] - ColorLib4[2][1]);
    err_g_b = abs(g_array_4[2] - ColorLib4[2][2]);
  }
 
  for (int i = 0 ; i < 10; i++ )
  {
    err_gr_r = abs(g_array_4[0] - ColorLib4[4][0]);
    err_gr_b = abs(g_array_4[1] - ColorLib4[4][1]);
    err_gr_g = abs(g_array_4[2] - ColorLib4[4][2]);
  }
  
  
  int center = (grayscale_lib_4[0] + grayscale_lib_4[4])/2;
  if(grayScale4>center)
  {
    return 1;
  }
  
  
  else if ( error_k <= thred1 && err_g_g > thred2 && err_r_r > thred2 && error_k < error_g)
  {
    //Serial.println("********* Black ***********");
    return 0;
  }

 
  else  if ( err_r_r <= 100 && err_r_g <= 100 && err_r_b <= 100 )
  {
    //Serial.println("********* Pink ************");
    return 4;
  }
  
  else if ( error_g <= thred1 )
  {
     if ( err_g_r <= 100 && err_g_g <= 100 && err_g_b <= 100 )
    {
      //Serial.println("********* Lightblue ************");
      return 3;
    }
    else if ( err_gr_r <= thred2 && err_gr_b <= thred2 && err_gr_g <= thred2 )
    {
      // Serial.println("********* Gray ************");
      return 0;
    }
  }  
  
  else
  {
    return 2;
  }
}





// Check Boundary
int checkBound_left(void)
{
  grayScale3 = (g_array_3[0] + g_array_3[1] + g_array_3[2])/3;
  delay(20);
  //Serial.print("bound1:");
  //Serial.println(grayScale3);
  grayScale4 = (g_array_4[0] + g_array_4[1] + g_array_4[2])/3;
  delay(20);
  //Serial.print("bound2:");
  //Serial.println(grayScale4);
  if(grayScale3<=boundThresh || grayScale4<=boundThresh)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}




int checkBound_right(void)
{
  grayScale1 = (g_array_1[0] + g_array_1[1] + g_array_1[2])/3;
  //Serial.print("bound1:");
  //Serial.println(grayScale1);
  grayScale2 = (g_array_2[0] + g_array_2[1] + g_array_2[2])/3;
  //Serial.print("bound2:");
  //Serial.println(grayScale2);
  if(grayScale1<=boundThresh || grayScale2<=boundThresh)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

// Fill the matrix of the defect map
void fillMatrix(void)
{
  defectMapArray_b[inquiryOrder[visitingIdx]] = sensor1;
  defectMapArray_b[inquiryOrder[visitingIdx+1]] = sensor2;
  defectMapArray_b[inquiryOrder[visitingIdx+2]] = sensor3;
  defectMapArray_b[inquiryOrder[visitingIdx+3]] = sensor4;
  
  defectMapArray_c[inquiryOrder[visitingIdx]] = sensor1_c;
  defectMapArray_c[inquiryOrder[visitingIdx+1]] = sensor2_c;
  defectMapArray_c[inquiryOrder[visitingIdx+2]] = sensor3_c;
  defectMapArray_c[inquiryOrder[visitingIdx+3]] = sensor4_c;
  
  visitingIdx = visitingIdx + 4;
}


int numDefect = 0;
//String inputString = "";
boolean stringComplete = false;
byte packet[83];
byte packet_c[83];
int serialState = 0;
byte randomID;
byte inChar;


void serialEvent() 
{
  while (Serial.available()) 
  {
    inChar = Serial.read(); 
    switch(serialState)
    {
      case 0:
      {
        if(inChar == 0xff)
        {
          serialState = 1;
        }
        break;
      }
      case 1:
      {
        randomID = inChar;
        serialState = 2;
        break;
      }
      case 2:
      {
        if(inChar == 1)
        {
          stringComplete = true;
        }
        serialState = 3;
        break;
      }
    }
  }
}



void sendArray()
{
  packet[0] = 81;
  //Serial.print("************sendArray**********");
  int defectCount = 0;
  for(int i=0;i<81;i++)
  {
    if(defectMapArray_b[i] != 0)
    {
      defectCount++;
    }
  }
  
  packet[1] = (byte)defectCount;

  for(int i=2;i<83;i++)
  {
    packet[i] = defectMapArray_b[i-2];
  }
  
  if(serialState == 3)
  {
    Serial.write(randomID);
    Serial.write(1);
    delay(3000);
    serialState = 0;
  }
  
//  for(int i=0; i<sizeof(packet); i++)
//  {  
//    Serial.print(packet[i]); 
//    Serial.print(' ');
//    if(i==1)
//    {
//      Serial.print('\n');
//    }
//    if(i%8==2)
//    {
//      Serial.print('\n');
//    }
//  }
  
  //Final output
  if (stringComplete) 
  {
    for(int i=0; i<sizeof(packet); i++)
    {  
      Serial.write(packet[i]); 
    }
    stringComplete = false;
  }
}         




void sendArray_c()
{
  packet_c[0] = 81;
  //Serial.print("************sendArray**********");
  int defectCount = 0;
  for(int i=0;i<81;i++)
  {
    if(defectMapArray_c[i] != 0 && defectMapArray_c[i] != 2)
    {
      defectCount++;
    }
  }
  
  packet_c[1] = (byte)defectCount;

  for(int i=2;i<83;i++)
  {
    packet_c[i] = defectMapArray_c[i-2];
  }
  
  if(serialState == 3)
  {
    Serial.write(randomID);
    Serial.write(1);
    delay(3000);
    serialState = 0;
  }
  
  for(int i=0; i<sizeof(packet_c); i++)
  {  
    Serial.print(packet_c[i]); 
    Serial.print(' ');
    if(i==1)
    {
      Serial.print('\n');
    }
    if(i%8==2)
    {
      Serial.print('\n');
    }
  }
  
  
  //Final output
  if (stringComplete) 
  {
    for(int i=0; i<sizeof(packet_c); i++)
    {  
      Serial.write(packet_c[i]); 
    }
    stringComplete = false;
  }
}         
/*******************************************/  
/*******************************************/  
/*************** Procedures ****************/  
/*******************************************/  
/*******************************************/

// Enter
void enterAndGetReady(void)
{
  while(1)
  {
    isBound = checkBound_left();
    if(isBound == 1)
    {
      break;
    }
    moveRight_enter();
  }
  
  //brake();
  
  int numStep_enter = 32;
  for(int i=0;i<numStep_enter;i++)
  {
    computeBodyAngle();
    moveRight_enter2();
  }
  
  //brake();
  
  turnLeft();
  
  //brake();
  
  traitAdjust(upAngle);
  
  //brake();
  
  startMove_back();
  while(1)
  {
    getDistFar();
    if(distFar<=upCheckPoint[0])
    {
      break;
    }
    computeBodyAngle();
    moveBackward_up(bodyAngle);
  }
  
  brake();
  
}

// Move downward all along until reach the ground
void moveDownAllAlong(void)
{
  
  traitAdjust(downAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(downAngle);
  
  //brake();
  
  startMove_down();
  while(1)
  {
    getDistNear();
    if(distNear<=downCheckPoint[1])
    {
      break;
    }
    computeBodyAngle();
    moveDownward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(downAngle);
  brake();
  
  // Defect detection ###  
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(downAngle);
  
  //brake();
  
  startMove_down();
  while(1)
  {
    getDistNear();
    if(distNear<=downCheckPoint[2])
    {
      break;
    }
    computeBodyAngle();
    moveDownward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(downAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(downAngle);
  
  //brake();
  
  startMove_down();
  while(1)
  {
    getDistNear();
    if(distNear<=downCheckPoint[3])
    {
      break;
    }
    computeBodyAngle();
    moveDownward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(downAngle);
  brake();
  
// Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(downAngle);
  
  //brake();
  
  startMove_down();
  while(1)
  {
    getDistNear();
    if(distNear<=downCheckPoint[4])
    {
      break;
    }
    computeBodyAngle();
    moveDownward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(downAngle);
  brake();
  
  // Defect detection ### 
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
}

// Move upward all along until reach the ceil
void moveUpAllAlong(void)
{
  
  traitAdjust(upAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(upAngle);
  
  startMove_up();
  while(1)
  {
    getDistFar();
    if(distFar>=upCheckPoint[1])
    {
      break;
    }
    computeBodyAngle();
    moveUpward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(upAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(upAngle);
  
  //brake();
  
  startMove_up();
  while(1)
  {
    getDistFar();
    if(distFar>=upCheckPoint[2])
    {
      break;
    }
    computeBodyAngle();
    moveUpward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(upAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(upAngle);
  
  //brake();
  
  startMove_up();
  while(1)
  {
    getDistFar();
    if(distFar>=upCheckPoint[3])
    {
      break;
    }
    computeBodyAngle();
    moveUpward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(upAngle);
  brake();
  
  // Defect detection ###
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
  //traitAdjust(upAngle);
  
  //brake();
  
  startMove_up();
  while(1)
  {
    getDistFar();
    if(distFar>=upCheckPoint[4])
    {
      break;
    }
    computeBodyAngle();
    moveUpward(bodyAngle);
  }
  
  //brake();
  
  traitAdjust(upAngle);
  brake();
  
  // Defect detection ### 
  sensor1 = checkDefect1_b();
  sensor2 = checkDefect2_b();
  sensor3 = checkDefect3_b();
  sensor4 = checkDefect4_b();
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.print(sensor4);
  Serial.print('\n');
  
//  sensor1_c = checkDefect1_c();
//  sensor2_c = checkDefect2_c();
//  sensor3_c = checkDefect3_c();
//  sensor4_c = checkDefect4_c();
//  Serial.print(sensor1_c);
//  Serial.print(sensor2_c);
//  Serial.print(sensor3_c);
//  Serial.print(sensor4_c);
//  Serial.print('\n');
  
  fillMatrix();
  
}

// U turns in the process
void downUturn(void)
{
  startMove_back();
  for(int i=0;i<numStep_back;i++)
  {
    computeBodyAngle();
    moveBackward_down(bodyAngle);
  }
  
  //brake();
  
  turnLeft();
  
  //brake();
  
  traitAdjust(rightAngle);
  
  //brake();
  
  startMove_up();
  for(int i=0;i<numStep_right_Uturn_down;i++)
  {
    computeBodyAngle();
    moveRight(bodyAngle);
  }
  
  //brake();
  
  turnLeft();
  
  //brake();
  
  traitAdjust(upAngle);
  
  //brake();
  
  startMove_back();
  while(1)
  {
    getDistFar();
    if(distFar<=upCheckPoint[0])
    {
      break;
    }
    computeBodyAngle();
    moveBackward_up(bodyAngle);
  }
  
  traitAdjust(upAngle);
  
  //brake();
  
}



void upUturn(void)
{
  turnRight();
  
  //brake();
  
  traitAdjust(rightAngle);
  
  //brake();
  
  startMove_up();
  for(int i=0;i<numStep_right_Uturn_up;i++)
  {
    computeBodyAngle();
    moveRight(bodyAngle);
  }
  
  //brake();
  
  turnRight();
  
  //brake();
  
  traitAdjust(downAngle);
  
  //brake();
  
  startMove_back();
  while(1)
  {
    getDistNear();
    if(distNear>=downCheckPoint[0])
    {
      break;
    }
    computeBodyAngle();
    moveBackward_down(bodyAngle);
  }
  
  traitAdjust(downAngle);
  
  //brake();
}



void endUturn(void)
{
  startMove_back();
  for(int i=0;i<numStep_back;i++)
  {
    computeBodyAngle();
    moveBackward_down(bodyAngle);
  }
  
  //brake();
  
  turnLeft();
  
  //brake();
  
  traitAdjust(rightAngle);
  
  //brake();
  
  while(1)
  {
    isBound = checkBound_right();
    if(isBound == 1)
    {
      break;
    }
    moveRight_enter();
  }
  
  //brake();
  
  int numStep_lastback = 30;
  for(int i=0;i<numStep_lastback;i++)
  {
    computeBodyAngle();
    moveLeft_reenter();
  }
  
  //brake();
  
  turnLeft();
  
  //brake();
  
  traitAdjust(upAngle);
  
  //brake();
  
  startMove_back();
  while(1)
  {
    getDistFar();
    if(distFar<=upCheckPoint[0])
    {
      break;
    }
    computeBodyAngle();
    moveBackward_up(bodyAngle);
  }
  
  //brake();
  
}

void endOfGame(void)
{ 
  digitalWrite(ambientLEDPin_high, HIGH);
  digitalWrite(ambientLEDPin_low, LOW);
  digitalWrite(systemLEDPin_high, LOW);
  digitalWrite(systemLEDPin_low, LOW);
  
  brake();
  
  delay(1000);

  for(int i=0;i<81;i++)
  {
    if(defectMapArray_b[i] == 1)
    {
      numDefect ++;
    }
  }
  
  digitalWrite(ambientLEDPin_high, LOW);
  digitalWrite(ambientLEDPin_low, LOW);
  digitalWrite(systemLEDPin_high, HIGH);
  digitalWrite(systemLEDPin_low, LOW);
}



/*******************************************/  
/*******************************************/  
/***************** Setup *******************/  
/*******************************************/  
/*******************************************/ 
void setup(void)
{
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  brake();
  Serial.begin(9600);
  
  TSC_Init();
  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callback,10);
  attachInterrupt(5, TSC_Count_1, RISING);
  attachInterrupt(4, TSC_Count_2, RISING);
  attachInterrupt(3, TSC_Count_3, RISING);
  attachInterrupt(2, TSC_Count_4, RISING);
  delay(2000);
  pinMode(LED_In1, OUTPUT);
  pinMode(LED_Out1, OUTPUT);
  pinMode(LED_In2, OUTPUT);
  pinMode(LED_Out2, OUTPUT);
  
  pinMode(ambientPower_high, OUTPUT);
  pinMode(ambientPower_low, OUTPUT);
  pinMode(ambientPin, INPUT);
    
  pinMode(ambientLEDPin_high, OUTPUT);
  pinMode(ambientLEDPin_low, OUTPUT);
  
  
  pinMode(systemLEDPin_high, OUTPUT);
  pinMode(systemLEDPin_low, OUTPUT);
  
  
  digitalWrite(LED_In1, LOW);
  digitalWrite(LED_Out1, LOW);
  digitalWrite(LED_In2, LOW);
  digitalWrite(LED_Out2, LOW);
  //Calibration();
  delay(2000);
}



/*******************************************/  
/*******************************************/
/*************** Main Loop *****************/  
/*******************************************/  
/*******************************************/  

int systemState = 0;

void loop(void)
{
  if(systemState == 0)
  {
    
  /*******************************************/  
  
    //getColorLibrary();
    
  /*******************************************/  
    
    ambientSwitch();
   
  /*******************************************/  
  
    enterAndGetReady();
    
  /*******************************************/  
    
    moveUpAllAlong();
  
  /*******************************************/  
  
    upUturn();
  
  /*******************************************/  
  
    moveDownAllAlong();
    
  /*******************************************/  
  
    downUturn();
  
  /*******************************************/  
  
    moveUpAllAlong();
    
  /*******************************************/  
  
    upUturn();
  
  /*******************************************/  
  
    moveDownAllAlong();
    
  /*******************************************/  
  
    endUturn();
    
  /*******************************************/  
  
    moveUpAllAlong();
  
  /*******************************************/  
  
    endOfGame();

  /*******************************************/  
    
    systemState = 1;
    
  }
  
  if(systemState == 1)
  {
    sendArray();
  }

}







