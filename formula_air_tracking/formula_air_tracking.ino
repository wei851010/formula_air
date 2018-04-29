#include<Servo.h>

#define SENSOR_PIN {A0, A1, A2, A3, A4}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10

#define CALIBRATION 2000 
#define STEERING_MAX 40
#define STEERING_MED 90
#define DELTA_T 0.001
#define SPEED 100

#define KP 0.03
#define KI 1 * DELTA_T
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

    for (int i = 0; i < 5; ++i) pinMode(sensor_pin[i], INPUT);
    brushless.attach(BLUSHLESS_PIN);
    steering.attach(STEERING_PIN);
    steering.write(STEERING_MED);

    brushless_init();

    //delay(3000);
    //test();
}

void loop() {
    line_follow();
    delay(DELTA_T *1000);
    if(Serial.available()) {
       char c = Serial.read();
    }
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
    long sensor_value[5] = {0};
    for  (int i = 0; i < 9 ; ++i)
        error_history[i] = error_history[i+1];

    long error = 0, weighted_sum = 0, sum = 0;
    Serial.print("Sensor value: \t");
    for (int i = 0; i < 5; ++i) {
        sensor_value[i] = analogRead(sensor_pin[i]);
        Serial.print(sensor_value[i]);
        Serial.print('\t');
        weighted_sum += sensor_value[i]*i*1000;
        sum += sensor_value[i];
    }

    error = weighted_sum/sum - CALIBRATION;
    Serial.print("\tError: ");
    Serial.print(error);
    error_history[9] = error;

    int error_i = 0, error_d = 0;
    error_d = error_history[9] - error_history[8];
    for (int i = 0; i < 10; ++i)
        error_i += error_history[i];

    int steering_cmd = STEERING_MED + KP * error + KI * error_i + KD * error_d;
    steering_cmd = constrain(steering_cmd, STEERING_MED-STEERING_MAX, STEERING_MED+STEERING_MAX);
    Serial.print("\tSteering cmd: ");
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
