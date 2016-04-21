//====Incluedes====
#include <Wire.h>
#include "ClickButton.h"
#include "Kalman.h"          
#include "VescController.h"
//=================

//====Wirering====
#define Poti1 0           //Analog Pin A0
#define Poti2 1           //Analog Pin A1
#define Poti3 2           //Analog Pin A2
#define ssr 12             //Digital D12
#define FootButton 5      //Digital Pin D5
#define PowerButtonPin 4  //Digital Pin D4
//=================

//====Objekte=====
ClickButton PowerButton(PowerButtonPin, HIGH);
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
VescController vesc(Serial);
//=================


//====Variabeln====

//for buttons
bool SystemOn = false;
int SystemOnButtonState = 0;
bool FootButtonPressed = false;

//for Poti
double valPoti1 = 0;
double valPoti2 = 0;
double valPoti3 = 0;

//IMU Data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only             <-------
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter            <-------
double rate;

float Pval, Ival, Dval;   //PID
float Accumulator=0;      //PID

#define RESTRICT_PITCH       // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define GAIN 27   

//i2c
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//PID Controll
float KP = 20;        
float KD = 0.1;        
float KI = 10;         
float output = 0.0;      
float absoutput = 0.0;

bool crash = false;

float accmaxX = 0;
float accmaxY = 0;
float accmaxZ = 0;

int maxAmps = 120;
int currentAmps = 0;
//=================



//#####################################################################################

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);    
  Wire.begin();

  i2cSetup();              //function with all the i2c and mpu-6050 setup

  //ButtonSetup
  PowerButton.debounceTime   = 20;   // Debounce timer in ms
  PowerButton.multiclickTime = 250;  // Time limit for multi clicks
  PowerButton.longClickTime  = 1000; // time until "held-down clicks" register
  pinMode(FootButton, INPUT);
  pinMode(13, OUTPUT);
  //SSR
  pinMode(ssr, OUTPUT);
  digitalWrite(ssr, LOW);   //Powerelectronics off
  digitalWrite(13, LOW);
}



//#####################################################################################

void loop() {
  

  //===PowerButton logic=============================
  PowerButton.Update();
  SystemOnButtonState = PowerButton.clicks;
  if (SystemOnButtonState <= -1 && SystemOn == false) 
  {
    SystemOn = true;
  }
  else if (SystemOnButtonState <= -1 && SystemOn == true)
  { 
   SystemOn = false;
  }
  //================================================= 
  
  
  if (SystemOn == true)
  {
    digitalWrite(ssr, HIGH);   //Powerelectronics on
    digitalWrite(13, HIGH);
    //KP = readPoti(Poti1, 40);        //0-40
    //KD = readPoti(Poti2, 2);        //0-2
    //KI = readPoti(Poti3, 10);        //0-10

//============================================================
    //checkForHighAcc(30000);    //Notaus bei zu hoher Beschleunigung
    if (digitalRead(FootButton) == HIGH)
    { 
      if (currentAmps < maxAmps)
      {
        currentAmps = currentAmps + 1;
      }
      getMPU6050Values();

      output = (PIDControll() / 400 ) * (-1); //0-1
      //output = (int)(output*100+0.5)/100.0;      //Auf 2 Nachkommastellen runden
      
      //if(output > 0.95) output = 0.95;
      //if(output > 20000) output = 20000;
      
      //if(output < -0.95) output = -0.95;
      //if(output < 500 && output > -500) output = 0;
      //if(output < 0.02 && output > -0.02) output = 0;
      
      output = output * currentAmps;

      vesc.SetCurrent(output);
      //vesc.SetDuty(output);
      //vesc.SetDuty(0.2)
      //vesc.SetRpm(output * 20000);
      //Serial.print(output); Serial.print("\t");
      //Serial.print("\r\n");
      //Serial.print(kalAngleX - 3.1); Serial.print("\t");

      //Serial.print(rate); Serial.print("\t");
      //Serial.print("\r\n");
      //Serial.print(currentAmps); Serial.print("\t");

      //getmaxacc();
    }
    else
    {
      currentAmps = 20;
      vesc.SetCurrent(0);
      //vesc.FullBrake();
    }
//============================================================

  //serialInfo();     //FootButton
  }  
  else
  {
    digitalWrite(ssr, LOW);   //Powerelectronics off
    digitalWrite(13, LOW);
    //Serial.print("System Off"); Serial.print("\t");
  }
  
  delay(50);
}





