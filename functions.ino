



float readPoti(int poti, int maxVal)
{
  float valPotiTemp = 0;
  valPotiTemp = analogRead(poti);       //0-891
  valPotiTemp = (valPotiTemp / 891) * maxVal;   //0-maxVal
  return valPotiTemp;
}

void printKValues(void)
{
    //Poti Print
  Serial.print("KD: "); Serial.print(KD,2); Serial.print("\t");
  Serial.print("KP: "); Serial.print(KP,2); Serial.print("\t");
  Serial.print("KI: "); Serial.print(KI,2); Serial.print("\t");
}


//===PID=============================================================================================

float PIDControll(void)
{
  float expectedTilt = 0;
  float currentTilt = kalAngleX - 3.1;
  float currentTiltRate = rate;

  float thrust = pid(currentTilt - expectedTilt, 1, currentTilt + currentTiltRate, KP, KI, KD );

  return thrust;
  /*
  Pval = KP * kalAngleX;
  Accumulator = Accumulator + kalAngleX*0.01;
  Accumulator = constrain(Accumulator, -10, 10);
  Ival = KI * Accumulator;
  Dval = KD * rate;
  output = (Pval + Ival + Dval);
  //--
  
  //OLD PID
  //output = ((kalAngleY - 9) * KP) + rate * KD + 120;   //gyroYrate
  
  // Clip as float (to prevent wind-up).
  if(output < -255.0) { output = -255.0; } 
  if(output > 255.0) { output = 255.0; }    
  
  //output = output * 1 / 255; // Output Werte von 0 bis 1
 */
}

float pid(float error, float integral, float derivative, float Kp, float Ki, float Kd)
{
  return -(error * Kp + integral * Ki + derivative * Kd);
}


//======ACC================================================================================

void getmaxacc(void)
{
  if (accX < accmaxX) accmaxX = accX;
  if (accY < accmaxY) accmaxY = accY;
  if (accZ < accmaxZ) accmaxZ = accZ;
         
  Serial.print("accmaxX: "); Serial.print(accmaxX); Serial.print("\t");
  Serial.print("accmaxY: "); Serial.print(accmaxY); Serial.print("\t");
  Serial.print("accmaxZ: "); Serial.print(accmaxZ); Serial.print("\t");
}

void checkForHighAcc(int maxacc)
{
  if (accX < -1 * maxacc || accX > maxacc || accY < -1 * maxacc || accY > maxacc || accZ < -1 * maxacc || accZ > maxacc) //Notaus bei zu hoher Beschleunigung
    {
      vesc.FullBrake();
      Serial.print("High acc!!"); Serial.print("\t");
      
      accmaxX = 0;
      accmaxY = 0;
      accmaxZ = 0;
      accX = 0;
      accY = 0;
      accZ = 0;  

      delay(5000);          //5s warten
      SystemOn = false;     //PowerElectronic turn off
    }
}

void LENotaus(void)
{
  digitalWrite(ssr, LOW);   //Powerelectronics off
  SystemOn = false;     //PowerElectronic turn off
}


void serialInfo(void)
{
  Serial.print("System On"); Serial.print("\t");    //StateInfo
  Serial.print(digitalRead(FootButton)); Serial.print("\t");
  printKValues();
}

