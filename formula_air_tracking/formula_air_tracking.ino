#include<Servo.h>

#define SENSOR_PIN {2, 3, 4, 5, 6}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10

#define CALIBRATION 1000 
#define STEERING_MAX 60
#define DELTA_T 0.001
#define SPEED 100

#define KP 10
#define KI 0 * DELTA_T
#define KD 0 / DELTA_T

Servo brushless;
Servo steering;

int brushless_cmd = 0;
int error_history[10] = {0};
int sensor_pin[5] = SENSOR_PIN;

void brushless_init();
void line_follow();
void test();

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < 5; ++i) pinMode(sensor_pin[i], OUTPUT);
    brushless.attach(BLUSHLESS_PIN);
    steering.attach(STEERING_PIN);
    steering.write(90);

    brushless_init();

    delay(3000);
    //test();
}

void loop() {
    
     
    line_follow();
    delay(DELTA_T *1000);
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
    for  (int i = 0; i < 9 ; ++i)
        error_history[i] = error_history[i+1];

    int error = 0, weighted_sum = 0, sum = 0;
    Serial.print("sensor value: ");
    for (int i = 0; i < 5; ++i) {
        sensor_value[i] = analogRead(sensor_pin[i]);
        Serial.print(sensor_value[i]+'\t');
        weighted_sum += sensor_value[i]*i*1000;
        sum += sensor_value[i];
    }
    Serial.println("");

    error = weighted_sum/sum - CALIBRATION;
    error_history[9] = error;

    int error_i = 0, error_d = 0;
    error_d = error_history[9] - error_history[1];
    for (int i = 0; i < 10; ++i)
        error_i += error_history[i];

    int steering_cmd = 90 + KP * error + KI * error_i + KD * error_d;
    steering_cmd = constrain(steering_cmd, 90-STEERING_MAX, 90+STEERING_MAX);
    Serial.print("steering cmd: ");
    Serial.println(steering_cmd);
    steering.write(steering_cmd); 


}

void brushless_init() {
    brushless_cmd = 180;
    brushless.write(brushless_cmd);
    delay(100);
    brushless_cmd = 15;
    brushless.write(brushless_cmd);
    delay(500);
}
