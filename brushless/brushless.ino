#include<Servo.h>

Servo brushless;

void brushless_init();
void setup() {
  brushless.attach(9);
  brushless_init();
}

void loop() {
    int cmd = 28;
    if (Serial.available())
        cmd = Serial.parseInt();
    brushless.write(cmd);
}

void brushless_init() {
  brushless.write(100);
  delay(100);
  brushless.write(20);
  delay(3000);
}

