#include<Servo.h>

#define SENSOR_PIN {2, 3, 4, 5, 6}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10

#define CALIBRATION {100, 100, 100, 100, 100}
#define STEERING_MAX 70
#define DELTA_T 0.001
#define SPEED 100

#define KP 10
#define KI 0 * DELTA_T
#define KD 0 / DELTA_T

Servo brushless;
Servo steering;

int brushless_cmd = 0;
int weighted_error[10] = {0};
int sensor_pin[5] = SENSOR_PIN;

void brushless_init();
void line_follow();
void test();

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < 5; ++i) pinMode(sensor_pin[i], OUTPUT);
    brushless.attach(BLUSHLESS_PIN);
    brushless_init();
    steering.attach(STEERING_PIN);
    steering.write(90);

    delay(3000);
    test();
}

void loop() {
    
     
//    line_follow();
//    delay(DELTA_T *1000);
}

void test() {
    brushless.write(SPEED);
    steering.write(40);
    /*
    int i = 90;
    for (; i >= 30; i -= 20) {
        Serial.println(i);
        steering.write(i);
        delay(1000);
    }
    for (; i <= 150; i += 20) {
        Serial.println(i);
        steering.write(i);
        delay(1000);
    }
    for (; i >= 90; i -= 20) {
        Serial.println(i);
        steering.write(i);
        delay(1000);
    }
    */
    delay(10000);
    brushless.write(0);

}

void line_follow() {
    int sensor_value[5] = {0};
    for  (int i = 9; i > 0 ; ++i)
        weighted_error[i] = weighted_error[i-1];

    int error = 0, sum = 0;
    for (int i = 0; i < 5; ++i) {
        sensor_value[i] = analogRead(sensor_pin[i]);
        error += sensor_value[i]*(i-2);
        sum += sensor_value[i];
    }

    error /= sum;
    weighted_error[0] = error;

    int error_i = 0, error_d = 0;
    error_d = weighted_error[0] - weighted_error[1];
    for (int i = 0; i < 10; ++i)
        error_i += weighted_error[i];

    int steering_cmd = KP * error + KI * error_i + KD * error_d;
    steering_cmd = constrain(steering_cmd, 90-STEERING_MAX, 90+STEERING_MAX);
    Serial.print(steering_cmd);
    steering.write(90 + steering_cmd); 


}

void brushless_init() {
    brushless_cmd = 180;
    brushless.write(brushless_cmd);
    delay(100);
    brushless_cmd = 15;
    brushless.write(brushless_cmd);
    delay(500);
}
