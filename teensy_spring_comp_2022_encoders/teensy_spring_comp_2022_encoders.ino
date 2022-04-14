#include <Encoder.h>

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include <TeensyTimerInterrupt.h>



TeensyTimer ITimer(TEENSY_TIMER_1);
#define HW_TIMER_INTERVAL_MS 5L

Encoder left(22,21);
Encoder right(17,18);
char msg[40];

void TimerHandler()
{
  sprintf(msg,"%019ld %019ld",left.read(),right.read());
//  Serial.print("Hello World: "); Serial.println(millis());
  Serial.println(msg);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
