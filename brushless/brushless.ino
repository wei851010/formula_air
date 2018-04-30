#include<Servo.h>

Servo brushless;

void brushless_init();
void setup() {
  brushless.attach(10);
  brushless_init();
}

void loop() {
  int cmd = map(analogRead(A0), 0, 1023, 0, 160);
  brushless.write(50);
}

void brushless_init() {
  brushless.write(100);
  delay(500);
  brushless.write(20);
  delay(3000);
}

