#include <SoftReset.h>
#include <Servo.h>
#include <Wire.h>
int raincount = 0;
//situations
bool nolight = false; //status wherein there is no available light
bool raining = false; //status wherin it is currently raining
bool noUV = false; //status wherein no sunlight is detected
bool toohot = false; //status where in the environment is too hot
bool freshreset = false; //status so that the system will know whether or not to reset
//situations
//timer
const unsigned long  PERIOD = 3600000; //hourly calibration
const long  test_timer = 15000; //15 second calibration
unsigned long target_time = 0L ; //reset timer
unsigned long test_target_time = 0L ; //reset timer
//timer
//servos
Servo servo; //360 degree motor
Servo servo2; //90 degree tilt
Servo servo3; //solar panel
//servos
int serialData = 0;
char input;
//ldrs
int ldrtopl = A12; //top left LDR 
int ldrtopr = A11; //top right LDR 
int ldrbotl = A13; // bottom left LDR 
int ldrbotr = A10; // bottom right LDR 
//ldrs
//vertical servo
int servoVerticalLimitHigh = 90;
int servoVerticalLimitLow = 30;
int servov = 30;
int servosolar=0; 
//vertical servo
// temperature
#include <DallasTemperature.h>
#include <OneWire.h>
#define ONE_WIRE_BUS 17
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// temperature
//lux meter
#include <BH1750FVI.h>
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
//lux meter
#include <Wire.h>
//lcd
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h" // Added library*
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the default I2C bus address
//lcd
//UV meter
int UVOUT = A14; //Output from the sensor
int REF_3V3 = A15; //3.3V power on the Arduino board
//UV meter
//rain sensor
const int rainsensor = 15;
//rain sensor

void setup(){
  //relay for whole system
  
  digitalWrite(23, HIGH);  
  delay(3000);
  digitalWrite(23, LOW);  
  pinMode(23, OUTPUT);
  Serial.begin(9600);
  //HIGH IS OFF AND LOW IS ON
  //relay for whole system
  //servo motor attachments
  servo.attach(9); //360 motor
  servo2.attach(12); // 90 degree tilt
  servo3.attach(14); // solar panel
  //servo motor attachments
  //button attachments
  pinMode(3, INPUT_PULLUP); //extend button
  pinMode(11, INPUT_PULLUP); //retract button
  pinMode(22, INPUT_PULLUP); //recalibrate button
  //button attachments
  //limit switch attachments
  pinMode(5, INPUT); //limit switch down
  pinMode(4, INPUT); //limit switch up
  pinMode(10, INPUT); //limit switch max counterclockwise
  pinMode(13, INPUT); //limit switch max clockwise
  //limit switch attachments
  //uv meter
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  //uv meter
  //lcd screen
  lcd.begin (20,4); // 20 x 4 LCD module
  lcd.setBacklightPin(3,POSITIVE); 
  lcd.setBacklight(HIGH);
  //lcd screen
  //lux sensor
  Wire.begin();
  LightSensor.begin();
  //lux sensor
  //temperature sensor
  sensors.begin();
  //temperature sensor
  //rain sensor
  pinMode(rainsensor, INPUT);
  //rain sensor
  //tilt servo to top
  servo2.write(30);
  //tilt servo to top 
}

void loop()
{  
  Serial.println(digitalRead(rainsensor));
  if(freshreset == false)
  {
  unsigned long currentMillis = millis(); //1 hour calibration
  if (currentMillis - test_target_time >= PERIOD) 
  {
      test_target_time = currentMillis;
      if(toohot != true && raining != true && noUV != true && nolight != true)
      {
        for(;;)
        {
            unsigned long currentMillis = millis(); //15 second calibration
            if (currentMillis - test_target_time >= test_timer) 
            {
              test_target_time = currentMillis;
              break;
            }
        suntracking();
        }
      }
   }
  sensorreading();
  checklimitswitches(); 
  checkbuttonpress();
  servocontrol(); 
  }
  else
  {
  checkbuttonpress();
  }
}


//lcd sensor updates
void sensorreading()
{
  //UV meter
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0);
  // UV meter
  
  //lux meter
  float lux = LightSensor.GetLightIntensity();
  // lux meter
  
  //temperature meter
  sensors.requestTemperatures();   
  //temperature meter  
   
  //lcd placements
  //lux sensor lcd
  lcd.setCursor(0,0);
  lcd.print("Light: ");
  lcd.print(lux);
  lcd.println(" lux    ");
  if (lux<10)
  {
    if(nolight == false)
    {
        servov = servo2.read();
      for(int angle = servov; angle >= 30; angle -= 1)  
        {                                  
          servo2.write(angle);               
          servov = angle;               
          delay(30);                       
        } 
      while (digitalRead(13)!=0)
        {
          servo.write(80);
        }
        servo.write(90);
        servosolar = servo3.read();
      for(int angle = servosolar; angle<=110; angle+=1)     
        {                                
          servo3.write(angle);             
          delay(30);                       
        } 
        nolight = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Not enought light");
        lcd.setCursor(0,1);
        lcd.print("Currently closing");
        delay(5000);
        freshreset = true;
        digitalWrite(23, HIGH);  
        return;
      }
  }
  else
  {
    nolight = false;
  }
   //lux sensor lcd

   //uv meter lcd
  lcd.setCursor(0,1);
  lcd.print("UV Rays:");
  lcd.print(uvIntensity); 
  lcd.print(" mW/cm");

  if(uvIntensity < .70f)
  {
    if(noUV == false)
    {
        servov = servo2.read();
      for(int angle = servov; angle >= 30; angle -= 1)    
        {                                  
          servo2.write(angle);    
          servov = angle;           
          delay(30);                       
        } 
        while (digitalRead(13)!=0)
        {
          servo.write(80);
        }
        servo.write(90);
        servosolar = servo3.read();
        for(int angle = servosolar; angle<=110; angle+=1)     
        {                                
          servo3.write(angle);           
          delay(30);                       
        } 
        noUV = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Not enough UV rays");
        lcd.setCursor(0,1);
        lcd.print("Currently closing");
        delay(5000);
        freshreset = true;
        digitalWrite(23, HIGH); 
        return;
      }
  }
  else
  {
    noUV = false;
  }
   //uv meter lcd
  //temperature lcd
  lcd.setCursor(0,2);
  lcd.print("Temperature:");
  lcd.print(sensors.getTempCByIndex(0)); 
  lcd.print("C  ");
  if(sensors.getTempCByIndex(0) > 55)
  {
    if(toohot == false)
    {
        servov = servo2.read();
      for(int angle = servov; angle >= 30; angle -= 1)  
        {                                  
          servo2.write(angle);              
          servov = angle;              
          delay(30);                       
        } 
        while (digitalRead(13)!=0)
        {
          servo.write(80);
        }
        servo.write(90);
        servosolar = servo3.read();
        for(int angle = servosolar; angle<=110; angle+=1)     
        {                                
          servo3.write(angle);             
          delay(30);                       
        } 
        toohot = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Temperatures too hot");
        lcd.setCursor(0,1);
        lcd.print("Currently closing");
        delay(5000);
        freshreset = true;
        digitalWrite(23, HIGH);  
        return;
      }
  }
  else
  {
    toohot = false;
  }
  //temperature lcd
  //rainsensor lcd
  
  if(digitalRead(rainsensor) == 0) 
    {
      delay(10000);
      if(digitalRead(rainsensor) == 0) 
        {
      if(raining == false)
      {
      lcd.setCursor(0,3);
      lcd.println("Currently raining"); 
        servov = servo2.read();
        
      for(int angle = servov; angle >= 30; angle -= 1)   
        {                                  
          servo2.write(angle);          
          servov = angle;               
          delay(30);                       
        } 
        while (digitalRead(13)!=0)
        {
          servo.write(80);
        }
        servo.write(90);
        servosolar = servo3.read();
        for(int angle = servosolar; angle<=110; angle+=1)   
        {                                
          servo3.write(angle);             
          delay(30);                       
        }
        
        raining = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Presently raining");
        lcd.setCursor(0,1);
        lcd.print("Currently closing");
        delay(5000);
        freshreset = true;
        digitalWrite(23, HIGH);  
        return;
      }
        }
    }
  else
    {
      raining = false;
      lcd.setCursor(0,3);
      lcd.println("No current rain   "); 
    }
  //rainsensor lcd
  //lcd placements
  
}
//lcd sensor updates

//stops 360 motor rotation
void stoprotate()
{
  servo.write(90);
}
//stops 360 motor rotation

//limit switches checker
void checklimitswitches()
{
  if(digitalRead(10) == 0) {
    stoprotate();
  }
  
  if(digitalRead(13) == 0) {
    stoprotate();
  }
}
//limit switches checker

// ldr sun tracking
void suntracking()
  {
      servov = servo2.read();  
      int topl = analogRead(ldrtopl);
      int topr = analogRead(ldrbotr);
      int botl = analogRead(ldrtopr);
      int botr = analogRead(ldrbotl);
      // calculating average
      int avgtop = (topl + topr) / 2; //average of top LDRs
      int avgbot = (botl + botr) / 2; //average of bottom LDRs
      int avgleft = (topl + botl) / 2; //average of left LDRs
      int avgright = (topr + botr) / 2; //average of right LDRs
      //averages
      int avgtilt;
      int avgrotate;
      if (avgtop < avgbot)
      {
          avgtilt = avgbot - avgtop;
          //up and down tilt
          if(avgtilt > 100)
          {
            servov = servov - 1;
            if (servov < servoVerticalLimitLow)
              {
                servov = servoVerticalLimitLow;
              }
            servo2.write(servov);
            delay(30);
          }
        }
        else if (avgbot < avgtop)
        {
          avgtilt = avgtop - avgbot;
          if(avgtilt > 100)
          {
          
          servov = servov + 1;
          if (servov > servoVerticalLimitHigh) 
           { 
            servov = servoVerticalLimitHigh;
           }
          servo2.write(servov);
          delay(30);
          }
        }
        else 
        {
          servo2.write(servov);
          Serial.println("perfect tilt");
        }
        /* ----------------------------------------------------------------------------------------------------------
        ------==========-----------========---------------------------------------------------------------------------
         //counterclockwise & clockwise*/
        if (avgleft > avgright)
        {
          avgrotate = avgleft - avgright;
          if(avgrotate > 100)
            {
            if (digitalRead(10) == 0)
              {
              servo.write(90);
              Serial.println("stopping counter clockwise");
              }
            else
              {
              servo.write(100);
              }
            Serial.println("rotating counter clockwise");
            
            }
        }
        else if (avgright > avgleft)
        {
          avgrotate = avgright - avgleft;
          if(avgrotate > 100)
            {
            if (digitalRead(13) == 0)
              {
              servo.write(90);
              Serial.println("stopping clockwise");
              }
            else
            {  
            servo.write(80);
            }
            Serial.println("rotating clockwise");
            }
        }
        else 
        {
          servo.write(90);
          Serial.println("perfect rotation");
        }
        
  }
  
// ldr sun tracking
  
void servocontrol()
{/*
  while (Serial.available() > 0) {
   char incomingCharacter = Serial.read();
   switch (incomingCharacter) 
   {
      case 'd': //extend solars
        for(int angle = 100; angle>=10; angle-=1)     // command to move from 180 degrees to 0 degrees 
        {                                
          servo3.write(angle);              //command to rotate the servo to the specified angle
          delay(30);                       
        } 
        break;

      case 'f': //retract solars
        for(int angle = 10; angle<=100; angle+=1)     // command to move from 180 degrees to 0 degrees 
        {                                
          servo3.write(angle);              //command to rotate the servo to the specified angle
          delay(30);                       
        } 
        break;
      
      case 'g': // tilt solar panels 
        for(int angle = servov; angle <= 90; angle += 1)    // command to move from 0 degrees to 180 degrees 
        {                                  
          servo2.write(angle);                 //command to rotate the servo to the specified angle
          servov = angle;
          delay(30);                       
        } 
        break;

      case 'h': // reset solar panels
        for(int angle = servov; angle >= 30; angle -= 1)    // command to move from 0 degrees to 180 degrees 
        {                                  
          servo2.write(angle);               //command to rotate the servo to the specified angle
          servov = angle;                //command to rotate the servo to the specified angle
          delay(30);                       
        } 
        break;
        
      case 'j':
        servo.write(80); // clockwise
        delay(500);
        break; 

      case 'k':
        servo.write(105); //counterclockwise
        delay(500);
        break; 
    }
}*/
}

//checks if button is being pressed
void checkbuttonpress()
{
  if(digitalRead(3)==0) //up
  {
    if(freshreset == true)
    {
    reset();
    freshreset = false;
    }
    Serial.println("UP BUTTON IS PRESSED");
    //solar panel extend
    servosolar = servo3.read();
    for(int angle = servosolar; angle>=10; angle-=1)     // command to move from 180 degrees to 0 degrees 
      {     
        servo3.write(angle);   
        if(angle == 10)
        {
          Serial.println("triggered track function");
          tensectracking();
        }           //command to rotate the servo to the specified angle
        delay(30);                      
      } 
    //solar panel extend
    digitalWrite(22,0);
    }
 
        
  else if(digitalRead(11)==0) // down
  {
      servov = servo2.read();
    for(int angle = servov; angle >= 30; angle -= 1)    // command to move from 0 degrees to 180 degrees 
      {                                  
        servo2.write(angle);               //command to rotate the servo to the specified angle
        servov = angle;                //command to rotate the servo to the specified angle
        delay(30);                       
      } 
      while (digitalRead(13)!=0)
      {
        servo.write(80);
      }
        servo.write(90);
    servosolar = servo3.read();
      for(int angle = servosolar; angle<=110; angle+=1)     // command to move from 180 degrees to 0 degrees 
      {                                
        servo3.write(angle);              //command to rotate the servo to the specified angle
        delay(30);                       
      } 
    digitalWrite(23, HIGH);
  }
  
  else if(digitalRead(22)==0) // recalibrate
  {
    if(toohot != true && raining != true && noUV != true && nolight != true)
    {
      for(;;)
      {
        unsigned long currentMillis = millis();
        if (currentMillis - test_target_time >= test_timer) {
          // save the last time you blinked the LED
          test_target_time = currentMillis;
          break;
        }
        suntracking();
        Serial.println("currently in recalibration suntracking");
      }
    }
  }
 }
//checks if button is being pressed

 //tracks the sun for 10 seconds
 void tensectracking()
 {      
  if(toohot != true && raining != true && noUV != true && nolight != true)
      {
  for(;;)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - test_target_time >= test_timer) {
        // save the last time you blinked the LED
        test_target_time = currentMillis;
        break;
      }
      suntracking();
      Serial.println("currently in function suntracking");
    }
      }
 }
 //tracks the sun for 10 seconds

//UV sensor stuff
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//UV sensor stuff

void(* resetFunc) (void) = 0;

void reset()
{ 
soft_restart();
  delay(100);
  Serial.println("never happens");
}
