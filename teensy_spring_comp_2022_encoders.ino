#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include <TeensyTimerInterrupt.h>
#include "Teensy_ISR_Timer.h"


TeensyTimer ITimer(TEENSY_TIMER_1);
#define HW_TIMER_INTERVAL_MS 50L

void TimerHandler()
{
  Serial.print("Hello World: "); Serial.println(millis());
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
