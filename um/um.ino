#include "GyverTimers.h"

int a[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

void setup() {
  Serial.begin(9600);
  // Устанавливаем период таймера 333000 мкс -> 0.333 c (3 раза в секунду)
  Timer2.setPeriod(300000);
  Timer2.enableISR();   // запускаем прерывание на канале А таймера 2
  pinMode(13, OUTPUT);  // будем мигать
}

void loop() {
  digitalWrite(13, 1);
  for (int i : a) {
    delay(500);
  }
  digitalWrite(13, 0);
  for (int i : a) {
    delay(500);
  }
}

// Прерывание А таймера 2
ISR(TIMER2_A) {
  Serial.println("isr!");
}