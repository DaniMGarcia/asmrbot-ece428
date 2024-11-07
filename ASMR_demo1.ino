// ECE 428 Embedded Systems
// ASMR Demo 1 Code

// DEFINE PINS
  // ultrasonic sensor
  int const trigPin = 10;
  int const echoPin = 9;
  // buzzer
  int const buzzerPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Duration will be the input pulse width and distance will be the distance to the obstacle in centimeters
  int duration, distance;
  
  // Output pulse with 1ms width on trigPin
  digitalWrite(trigPin, HIGH);
  delay(1);
  digitalWrite(trigPin, LOW);
  
  // Measure the pulse input in echo pin
  duration = pulseIn(echoPin, HIGH);
  // Distance is half the duration devided by 29.1 (from datasheet)
  distance = (duration/2) / 29.1;
  
  // if distance less than 10 centimeters and more than 0 (0 or less means over range)
  if (distance <= 10 && distance >= 0) {
    // Buzz/beep
    tone(buzzerPin, 1000);
  } else {
    // Don't buzz/beep
    noTone(buzzerPin);
  }

  // Wait 60 ms
  delay(60);

}
