#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ECE 428 Embedded Systems
// ASMR Demo 2 Code

// DEFINE Pins
  // Ultrasonic sensor
  int const trigPin = 10; // sensor in pin 10 of Arduino
  int const echoPin = 9;  // sensor in pin 9 of Arduino
  
  // buzzer
  int const buzzPin = 2;  // buzzer connected to pin 2 of Arduino

  // LCD module
  LiquidCrystal_I2C lcd(0x27, 16, 2);
  int distanceLCD;
  long durationLCD;
  // A4 & A5
  
  // PIR sensor
 /* int PIRpin = 4; // PIR in pin 4 of Arduino
  int PIRstate = LOW;
*/
  int PIRpin[] = {4}; // PIR pin numbers
  int currentPIRpin = 4; // the current PIR pin; begin with the first in the sequence above
  int PIRprevState[] = {1,1,1}; // the previous state of the PIR (0 = LOW, 1 = HIGH)
  int PIRposition[] = {0,90,180}; // assign angles for servo motor (0-157 distributed equally between 5 PIR sensors)
  boolean PIRstatus; // Set status of PIR sensor as either true or false
  
  // DC Motor
  const int ENA_PIN = 7;
  const int IN1_PIN = 6;
  const int IN2_PIN = 5;
  

void setup() {
  // put your setup code here, to run once:
  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // buzzer
  pinMode(buzzPin, OUTPUT);

  // LCD
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  Serial.begin(9600); 

  // PIR 
  pinMode(PIRpin, INPUT);

  // DC motor
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  for (int p = 0; p < 1; p++)  { // set all PIR sensors as INPUTS
    pinMode(PIRpin[p], INPUT);
  } // end 'p' for
  Serial.print("Calibrating PIR Sensors ");
  for(int c = 0; c < 10; c++){ // calibrate PIR sensors for 15 seconds (change from 10-60 sec depending on your sensors)
      Serial.print(".");
      delay(1000); // wait 1 second
  } // end calibration for
  Serial.println("PIR Sensors Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  SensorToLCD();
  UStoBuzz();
  PIRtoMotors();
}

// ultrasonic sensor to buzzer
void UStoBuzz(){ // add more than just turn on and off - maybe change the tone or frequency
  int durationUS, distanceUS;
  
  // Output pulse with 1ms width on trigPin
  digitalWrite(trigPin, HIGH);
  delay(1);
  digitalWrite(trigPin, LOW);
  
  // Measure the pulse input in echo pin
  durationUS = pulseIn(echoPin, HIGH);
  // Distance is half the duration devided by 29.1 (from datasheet)
  distanceUS = (durationUS/2) / 29.1;
  
  // if distance less than 10 centimeters and more than 0 (0 or less means over range)
  if (distanceUS <= 10 && distanceUS >= 0) {
    // Buzz/beep
    tone(buzzPin, 1000);
  } else {
    // Don't buzz/beep
    noTone(buzzPin);
  }

  // Wait 60 ms
  delay(30);

  
}

// ultrasonic & PIR sensors to LCD
void SensorToLCD(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  durationLCD = pulseIn(echoPin, HIGH);
  distanceLCD = durationLCD * 0.0340 / 2;
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  Serial.print(distanceLCD);
  lcd.print(distanceLCD);
  lcd.print(" cm");
  lcd.setCursor(0, 1);
  lcd.print("    ");



  for (int PIR = 0; PIR < 1; PIR++) { // start this loop for each PIR sensor
    currentPIRpin = PIRpin[PIR]; // set current PIR pin to current number in 'for' loop

    if(digitalRead(PIRpin[PIR]) == HIGH){ // if motion is detected
      lcd.print("Motion Detected!");
    }
    else{ // if motion is NOT detected
      lcd.print("Roaming...");
    }
  }  
  
 delay(10); // delay(1000);
}

void PIRtoMotors(){
 
  //digitalWrite(IN1_PIN, HIGH);
  //digitalWrite(IN2_PIN, LOW);

  for (int PIR = 0; PIR < 1; PIR++) { // start this loop for each PIR sensor
    currentPIRpin = PIRpin[PIR]; // set current PIR pin to current number in 'for' loop

    if(digitalRead(PIRpin[PIR]) == LOW){ // if motion is not detected
      Serial.println("Roaming...");
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, HIGH);
      analogWrite(ENA_PIN, 255);
      Serial.println("Motors are on");
      // delay(5000);
    }
    else if(digitalRead(PIRpin[PIR] == HIGH)){ // if motion is detected
      Serial.println("Motion detected!");
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, LOW);
      analogWrite(ENA_PIN, 0);
      Serial.println("Motors have stopped!");
    }
  }  


 // if(PIRstate == LOW) { // if motion is not detected, move
    //analogWrite(ENA_PIN, 0);  
 // }
 // else if(PIRstate == HIGH){ // if motion is detected, stop
  //  analogWrite(ENA_PIN, 255);
 // }
  
 /* for (int speed = 0; speed <= 255; speed++) {
    analogWrite(ENA_PIN, speed); // control the speed
    delay(10);
  } 

  delay(1000); // rotate at maximum speed 1 seconds in in clockwise direction

  // change direction
  digitalWrite(IN1_PIN, LOW);   // control motor A spins anti-clockwise
  digitalWrite(IN2_PIN, HIGH);  // control motor A spins anti-clockwise

 // delay(1000); // rotate at maximum speed 1 seconds in in anti-clockwise direction

  for (int speed = 255; speed >= 0; speed--) {
    analogWrite(ENA_PIN, speed); // control the speed
    delay(10);
  }

  delay(60); // stop motor 1 second
*/

  
}
