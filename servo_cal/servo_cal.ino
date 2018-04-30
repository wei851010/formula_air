#include<Servo.h>
Servo s;

void setup() {
    s.attach(10);
    s.write(30);
}
void loop() {
}
