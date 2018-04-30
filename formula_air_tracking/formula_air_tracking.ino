//#define ADAPTIVE
#define DIGITAL

#include<Servo.h>

#define SENSOR_PIN {A0, A1, A2, A3, A4}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10

#define CALIBRATION 2000 
#define STEERING_MAX 60
#define STEERING_MED 90
#define DELTA_T 0.001
#define SPEED 35 

#ifdef ADAPTIVE
#define LR 0.0000001
#endif

#define KP 6 * 0.001 
#define KI 0.8 * 0.001
#define KD 12 * 0.001

Servo brushless;
Servo steering;

int brushless_cmd = 0;
int error_history[10] = {0};
int sensor_pin[5] = SENSOR_PIN;
int pre_sensor_value = {0};
float kp = KP, ki = KI, kd = KD;

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
    //test();
}

void loop() {
    line_follow();
    delay(DELTA_T *1000);
}

void line_follow() {
    long sensor_value[5] = {0};
    for  (int i = 0; i < 9 ; ++i)
        error_history[i] = error_history[i+1];

    long error = 0, weighted_sum = 0, sum = 0;
    Serial.print("Sensor value:");
    for (int i = 0; i < 5; ++i) {
#ifndef DIGITAL
        sensor_value[i] = analogRead(sensor_pin[i]);
        //if (sensor_value[i] > 400 && !pre_sensor_value[i] || sensor_value[i] < 300 && pre_sensor_value[i])
        //    sensor_value[i] = !sensor_value[i];
#else
        sensor_value[i] = (analogRead(sensor_pin[i]) > 400);
#endif
        Serial.print('\t');
        Serial.print(sensor_value[i]);
        weighted_sum += sensor_value[i]*i*1000;
        sum += sensor_value[i];
    }
    error = weighted_sum/sum - CALIBRATION;
    if (!sum) {
        error = (abs(error_history[8]) >= 2000)? constrain(error_history[8]*1.5, -3000, 3000): error_history[8];
    }
    Serial.print("\tSum: ");
    Serial.print(sum);
    Serial.print("\tError: ");
    Serial.print(error);
    error_history[9] = error;

    int error_i = 0, error_d = 0;
    error_d = error_history[9] - error_history[8];
    for (int i = 0; i < 10; ++i)
        error_i += error_history[i];

    int steering_cmd = STEERING_MED + kp * error + ki * error_i + kd * error_d;
    steering_cmd = constrain(steering_cmd, STEERING_MED-STEERING_MAX, STEERING_MED+STEERING_MAX);
    Serial.print("\tSteering cmd: ");
    Serial.print(steering_cmd);
    steering.write(steering_cmd); 

#ifdef ADAPTIVE
    //kp -= -LR * error * error;
    ki -= -LR * error * error_i;
    kd -= -LR * error * error_d;
    Serial.print("\tkp: ");
    Serial.print(kp);
    Serial.print("\tki: ");
    Serial.print(ki);
    Serial.print("\tkd: ");
    Serial.print(kd);
#endif
    Serial.println("");
}

void brushless_init() {
    brushless.write(100);
    delay(100);
    brushless.write(15);
    delay(3000);
    brushless.write(SPEED);
}
