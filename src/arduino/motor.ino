  #include <PID_v1.h>
  
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
  
  //PID
  double originalSetpoint = 0;
  double setpoint = originalSetpoint;
  double movingAngleOffset = 0.1;
  double input, output;

  //PID constants
  double Kp = 50;   
  double Kd = 0;
  double Ki = 0;
  PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
  
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
    
     //setup PID
   pid.SetMode(AUTOMATIC);
   pid.SetSampleTime(10);
   pid.SetOutputLimits(-255, 255); 
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
     	String val = Serial.readStringUntil('\n');
        val.toCharArray(buffer, 8);
        input = atof(buffer);
        pid.Compute();
        if (output > 0) {
          forward(output);
        } else {
          reverse(output);
        }
  }
