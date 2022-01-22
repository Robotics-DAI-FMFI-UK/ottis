#include <Servo.h>

// pin connections:
// servos:
//    D3 LEFT FOOT
//    D5 LEFT KNEE
//    D6 LEFT HIP
//    D7 LEFT SHOULDER
//    D8 LEFT ARM
//    D9 CHEST
//    D10 RIGHT SHOULDER
//    D11 RIGHT ARM
//    D12 RIGHT HIP
//    D14/A0  RIGHT KNEE
//    D15/A1  RIGHT FOOT

#define LEFT_FOOT 3
#define LEFT_KNEE 5
#define LEFT_HIP 6
#define LEFT_SHOULDER 7
#define LEFT_ARM 8
#define CHEST 9
#define RIGHT_SHOULDER 10
#define RIGHT_ARM 11
#define RIGHT_HIP 12
#define RIGHT_KNEE 14
#define RIGHT_FOOT 15


Servo s;
int state;

void setup() {
  state = 0;
  s.attach(RIGHT_FOOT);
  s.write(90);
  Serial.begin(9600);
}

void loop() {

  if (Serial.available())
  {
    char c = Serial.read();
    state = (state + 1) % 3;
  }

  if (state == 0) s.write(90);
  else if (state == 1) s.write(175);
  else if (state == 2) s.write(5);
    
  
}
