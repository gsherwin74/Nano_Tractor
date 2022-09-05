#include <Arduino.h>
#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
int pwmOutput = 20;         //Value for engine sound
int pwmSpeed = 0;           //Value for motor speed
int pwmSpeedTarget = 0;     //Goal to reach for speed
bool FORWARD = true;        //Value for if we are going forward
int SpeedDelay = 5;        //Delay to make speed adjustment

//**********  Inputs  ******************
const int THROTTLE = 12;    //Inupt for throttle switch
const int GEAR_2 = 8;       //Input for 2nd gear selector
const int GEAR_1 = 7;       //Input for 1st gear selector
const int GEAR_R = 2;       //Input for reverse gear selector

//**********  Outputs  ******************
const int MOTOR_L_F = 11;   //Output for left motor forward
const int MOTOR_L_R = 10;   //Output for left motor reverse
const int MOTOR_R_F = 6;    //Output for right motor forward
const int MOTOR_R_R = 5;    //Output for right motor reverse
const int MAIN_RELAY = 9;   //Output for main power relay
const int AUX = 4;          //Output for backup beeper
const int ENGINE_SOUND = 3; //Output for enigne sound module

void setup() {
  
  ESC.attach(ENGINE_SOUND,1000,2000); // Attach the ESC on pin 2 (pin, min pulse width, max pulse width in microseconds)

  pinMode(THROTTLE, INPUT_PULLUP);
  pinMode(GEAR_2, INPUT_PULLUP);
  pinMode(GEAR_1, INPUT_PULLUP);
  pinMode(GEAR_R, INPUT_PULLUP);

  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_R, OUTPUT);
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_R, OUTPUT);
  pinMode(MAIN_RELAY, OUTPUT);
  pinMode(AUX, OUTPUT);

  Serial.begin(9600);
  
  // Start the engine and get ready
  pwmOutput = 20;
  ESC.write(pwmOutput);
  pwmSpeed = 0;
  digitalWrite(MOTOR_L_R,pwmSpeed);
  digitalWrite(MOTOR_R_R,pwmSpeed);
  digitalWrite(MOTOR_L_F,pwmSpeed);
  digitalWrite(MOTOR_R_F,pwmSpeed);
  digitalWrite(MAIN_RELAY,255);
}

void loop() {
  int EngineValue = digitalRead(THROTTLE);
  
  if (EngineValue == LOW)   //***************************   PEDAL IS PRESSED   ************************************************
  {
    if (digitalRead(GEAR_2) == LOW)  //***************************   2ND GEAR   ************************************************    
    {
      if (!(FORWARD))  //We are supposed to be going forward but are not
      {
        if (pwmSpeed > 0) //We need to slow to a stop before changing direction
        {
          pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
          digitalWrite(MOTOR_L_R,pwmSpeed); //send new value to left motor reverse
          digitalWrite(MOTOR_R_R,pwmSpeed); //send new value to right motor reverse
          pwmOutput = 20;
          ESC.write(pwmOutput);
          delay(SpeedDelay);  //give time to make speed change
        } else  //We are stopped
        {
          FORWARD = !(FORWARD); //set direction to forward now
        }
      } else  //speed up to 2nd gear speed
      {
        pwmSpeedTarget = 255; //Set target to full speed
        if (pwmSpeed < pwmSpeedTarget)  //We have not reached target speed yet
        {
          pwmSpeed = pwmSpeed + 1;  //Increase
          digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
          digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
          pwmOutput = 80;
          ESC.write(pwmOutput);
          delay(SpeedDelay);  //give time to make speed change
        }
      } 
    } else  //We are not in 2nd gear
    {
      if (digitalRead(GEAR_R) == LOW)  //***************************   REVERSE GEAR   ************************************************  
      {
        if ((FORWARD))  //We are supposed to be going reverse but are not
        {
        if (pwmSpeed > 0) //We need to slow to a stop before changing direction
        {
          pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
          digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
          digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
          pwmOutput = 20;
          ESC.write(pwmOutput);
          delay(SpeedDelay);  //give time to make speed change
        } else  //We are stopped
        {
          FORWARD = !(FORWARD); //set direction to forward now
        }
        } else  //Speed up to reverse speed
        {
        pwmSpeedTarget = 127; //Set target to full speed
        if (pwmSpeed < pwmSpeedTarget)  //We have not reached target speed yet
          {
            pwmSpeed = pwmSpeed + 1;  //Increase speed
            digitalWrite(MOTOR_L_R,pwmSpeed); //send new value to left motor reverse
            digitalWrite(MOTOR_R_R,pwmSpeed); //send new value to right motor reverse
            pwmOutput = 50;
            ESC.write(pwmOutput);
            delay(SpeedDelay);  //give time to make speed change
          }
        } 
      } else  //***************************   1ST GEAR   ************************************************  
      {
        if (!(FORWARD))  //We are supposed to be going forward but are not
        {
          if (pwmSpeed > 0) //We need to slow to a stop before changing direction
          {
            pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
            digitalWrite(MOTOR_L_R,pwmSpeed); //send new value to left motor reverse
            digitalWrite(MOTOR_R_R,pwmSpeed); //send new value to right motor reverse
            pwmOutput = 20;
            ESC.write(pwmOutput);
            delay(SpeedDelay);  //give time to make speed change
          } else  //We are stopped
          {
            FORWARD = !(FORWARD); //set direction to forward now
          }
        } else  //speed up to 1st gear speed
        {
          pwmSpeedTarget = 127; //Set target to half speed
          if (pwmSpeed < pwmSpeedTarget)  //We have not reached target speed yet
          {
            pwmSpeed = pwmSpeed + 1;  //Increase speed
            digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
            digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
            pwmOutput = 50;
            ESC.write(pwmOutput);
            delay(SpeedDelay);  //give time to make speed change
          } else
          {
             pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
            digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
            digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
            pwmOutput = 50;
            ESC.write(pwmOutput);
            delay(SpeedDelay);  //give time to make speed change
          }
        } 
      }
    }
  } else  //***************************   PEDAL IS NOT PRESSED   ************************************************    
  {
    if (digitalRead(GEAR_2) == LOW)  //We are in 2nd gear
    {
      if (pwmSpeed > 0) //We need to slow to a stop
        {
          pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
          digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
          digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
          pwmOutput = 20;
          ESC.write(pwmOutput);
          delay(SpeedDelay);  //give time to make speed change
        }
    } else
    {
      if (digitalRead(GEAR_R) == LOW)  //We are in reverse
      {
        if (pwmSpeed > 0) //We need to slow to a stop 
        {
          pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
          digitalWrite(MOTOR_L_R,pwmSpeed); //send new value to left motor reverse
          digitalWrite(MOTOR_R_R,pwmSpeed); //send new value to right motor reverse
          pwmOutput = 20;
          ESC.write(pwmOutput);
          delay(SpeedDelay);  //give time to make speed change          
        } 
      }else  //By default we are in 1st gear
      {
          if (pwmSpeed > 0) //We need to slow to a stop
          {
            pwmSpeed = pwmSpeed - 1;  //decrease to the current speed
            digitalWrite(MOTOR_L_F,pwmSpeed); //send new value to left motor forward
            digitalWrite(MOTOR_R_F,pwmSpeed); //send new value to right motor forward
            pwmOutput = 20;
            ESC.write(pwmOutput);
            delay(SpeedDelay);  //give time to make speed change
          }
       }
    }
  }
  Serial.print("Throttle: ");
  Serial.print(EngineValue);
  Serial.print(" Forward: ");
  Serial.print(FORWARD);
  Serial.print(" 1st: ");
  Serial.print(digitalRead(GEAR_1));
  Serial.print(" 2nd: ");
  Serial.print(digitalRead(GEAR_2));
  Serial.print(" Reverse: ");
  Serial.print(digitalRead(GEAR_R));
  Serial.print(" pwmSpeedTarget: ");
  Serial.print(pwmSpeedTarget);
  Serial.print(" pwmSpeed: ");
  Serial.println(pwmSpeed);

}