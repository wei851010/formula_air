#include<Servo.h>

Servo brushless;

void brushless_init();
void setup() {
  brushless.attach(9);
  brushless_init();
}

void loop() {
  int cmd = map(analogRead(A0), 0, 1023, 0, 160);
  brushless.write(cmd);
}

void brushless_init() {
  brushless.write(180);
  delay(500);
  brushless.write(15);
  delay(500);
}

