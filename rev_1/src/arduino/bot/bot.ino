  // Title: Motor controller sketch for Self Balancing Bot
  // Author: Manivannan Sadhasivam
  // Copyright (c) 2017 Linaro Ltd.
  // All rights reserved.


  // connect motor controller pins to Arduino digital pins

  // motor one
  int EN_A = 10;
  int IN1 = 9;
  int IN2 = 8;
  
  // motor two
  int EN_B = 5;
  int IN3 = 7;
  int IN4 = 6;
  
  char buffer[10];
  
  void setup()
  {
    Serial.begin(9600);
    // set all the motor control pins to outputs
    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
  }
  
  void forward(float duty_cycle)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN_A, duty_cycle);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN_B, duty_cycle);
  }
  
  void reverse(float duty_cycle)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN_A, duty_cycle);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN_B, duty_cycle);
  }
  
  void loop() {
     String dir = Serial.readStringUntil(':');
     if(dir != ""){
     	String val = Serial.readStringUntil('\n');
        val.toCharArray(buffer, 10);
     }
     if (dir == "f")
        forward(atof(buffer));
     else if (dir == "r")
        reverse(atof(buffer));
  }
