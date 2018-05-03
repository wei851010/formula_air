//#define ADAPTIVE
#define DIGITAL
#define SERIAL_DEBUG

#include<Servo.h>

#define SENSOR_PIN {A0, A1, A2, A3, A4, A5}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10

#define SET_POINT 2000 
#define STEERING_MAX 60
#define STEERING_MED 90
#define DELTA_T 0.001
#define SPEED 28 
#define CALIBRATION {{400, 550}, {450, 600}, {450, 600}, {450, 600}, {400, 550}, {550, 700}}

#ifdef ADAPTIVE
#define LR 0.0000001
#endif

#define KP 2* 0.001 
#define KI 0.2 * 0.001
#define KD 3 * 0.001

Servo brushless;
Servo steering;

float weight[6] = {-5, -3, -1, 1, 3, 5};
int brushless_cmd = 0;
int error = 0;
long error_sum = 0;
int error_last = 0;
int sensor_pin[6] = SENSOR_PIN;
int calibration[6][2] = CALIBRATION;
int pre_sensor_value[6] = {0};
float kp = KP, ki = KI, kd = KD;
int counter = 0;

void brushless_init();
void line_follow();
int pid();

void setup() {
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    for (int i = 0; i < 6; ++i) pinMode(sensor_pin[i], INPUT);
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
    long sensor_value[6] = {0};

    long weighted_sum = 0, sum = 0;
#ifdef SERIAL_DEBUG
    Serial.print("Sensor value:");
#endif
    for (int i = 0; i < 6; ++i) {
#ifndef DIGITAL
        sensor_value[i] = analogRead(sensor_pin[i]);
#else
        sensor_value[i] = analogRead(sensor_pin[i]);
#ifdef SERIAL_DEBUG
        Serial.print(' ');
        Serial.print(sensor_value[i]);
#endif
        if (sensor_value[i] > calibration[i][1] && !pre_sensor_value[i] || sensor_value[i] < calibration[i][0] && pre_sensor_value[i])
            sensor_value[i] = !pre_sensor_value[i];
        else
            sensor_value[i] = pre_sensor_value[i];
        pre_sensor_value[i] = sensor_value[i];
#endif
#ifdef SERIAL_DEBUG
        Serial.print(' ');
        Serial.print(sensor_value[i]);
#endif
        weighted_sum += sensor_value[i]*weight[i]*1000;
        sum += sensor_value[i];
    }
    error = weighted_sum/sum;
    if (!sum) {
        error = (abs(error_last) >= 5000)? constrain(error_last*1.5, -10000, 10000): error_last;
    }


#ifdef SERIAL_DEBUG
    Serial.print("\tSum: ");
    Serial.print(sum);
    Serial.print("\tError: ");
    Serial.print(error);
#endif

    int steering_cmd = pid();
#ifdef SERIAL_DEBUG
    Serial.print("\tSteering cmd: ");
    Serial.print(steering_cmd);
#endif
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
#ifdef SERIAL_DEBUG
    Serial.println("");
#endif
}

int pid() {
    error_sum = error_sum*0.9 + error;
    long error_p = error;
    long error_d = error - error_last;
    long error_i = error_sum;
    counter = (error_d)? 0: counter+1;
    if (counter >= 500)
        brushless.write(SPEED+5);
    else
        brushless.write(SPEED);

    int steering_cmd = STEERING_MED + error_p * kp + error_i * ki + error_d * kd;
    Serial.print(error_d*kd);
    steering_cmd = constrain(steering_cmd, STEERING_MED-STEERING_MAX, STEERING_MED+STEERING_MAX);
    error_last = error;
    
    return steering_cmd;
}

void brushless_init() {
    brushless.write(100);
    delay(100);
    brushless.write(15);
    delay(3000);
    brushless.write(SPEED);
}
