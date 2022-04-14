#include <Servo.h>

#include <TimerOne.h>

#define SENSOR_0 A0
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3

//volatile char send_msg[21];
char incoming_msg[3];
//int read_bytes;

Servo Lservo;
Servo Rservo;

void setup() {
  Lservo.attach(4);
  Rservo.attach(5);
// read_bytes = 0;
 Serial.begin(115200);
 Timer1.initialize(100000);
 Timer1.attachInterrupt(printValues);
}

void printValues() {
  char send_msg[21];
  sprintf(send_msg,"%04d %04d %04d %04d",analogRead(SENSOR_0),analogRead(SENSOR_1),analogRead(SENSOR_2), analogRead(SENSOR_3));
  Serial.println((send_msg));
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(send_msg);
  if (Serial.available() > 9) {
//     long left = Serial.parseInt();
//     long right = Serial.parseInt();
     Lservo.write(Serial.parseInt());
     Rservo.write(Serial.parseInt());
     }
}
  
