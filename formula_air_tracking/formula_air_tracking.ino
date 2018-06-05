/************************************************
    FileName    [ formula_air_tracking.ino ]
    Synopsis    [ Tracking line using PID and bang bang control ]
    Author      [ Johnson Shih, Frank Lin ]
    Copyleft    [ Group 15 小萌牛©2018, NTUME ]
************************************************/

/********************Options********************/
#define DIGITAL
#define SERIAL_DEBUG
//#define BOOSTING
/********************Include********************/
#include <Servo.h>
#include <NewPing.h>
/********************Pins***********************/
#define SENSOR_PIN {A0, A1, A2, A3, A4, A5}
#define BLUSHLESS_PIN 9
#define STEERING_PIN 10
#define TRIGGER_PIN 11
#define ECHO_PIN 12
/********************Other Parameters***********/
#define STEERING_MAX 35
#define STEERING_MED 90
#define DELTA_T 1
#define SPEED 28
#define BARRIER_DELAY 900
#define BARRIER_DIST 60
#define MAX_DISTANCE 100
/********************Calibration Values*********/
#define CALIBRATION {{550, 650}, {630, 730}, {630, 730}, {550, 650}, {600, 700}, {750, 850}}
/********************PID Parameters*************/
#define KP 4.2 * 0.001 
#define KI 0.01 * 0.001
#define KD 6 * 0.001

Servo brushless;
Servo steering;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
/********************Variables******************/
float weight[6] = {-5, -3, -1, 1, 3, 5};
int brushless_cmd = 0;
int error = 0;
long error_sum = 0;
int error_last = 0;
int sensor_pin[6] = SENSOR_PIN;
int calibration[6][2] = CALIBRATION;
int pre_sensor_value[6] = {0};
int dist_history[5] = {0};
unsigned long start_time = 0;
float kp = KP, ki = KI, kd = KD;
enum Status {normal, barrier};
Status state = normal;
#ifdef BOOSTING
int counter = 0;
#endif
/********************Functions******************/
void brushless_init();
void line_follow();
int pid();
void skip_barrier();
int get_dist();
int ultrasonic();

void setup() {
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    //Pin initialization
    for (int i = 0; i < 6; ++i) pinMode(sensor_pin[i], INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    brushless.attach(BLUSHLESS_PIN);
    steering.attach(STEERING_PIN);
    //Intial position
    steering.write(STEERING_MED);
    state = normal;
    brushless_init();
}

void loop() {
    if (get_dist() < BARRIER_DIST) {
      start_time = millis();
      state = barrier;
    }
    if (state == normal)
        line_follow();
    else
        skip_barrier();
    delay(DELTA_T);
#ifdef SERIAL_DEBUG
    Serial.println("");
#endif
}

void line_follow() {
    long sensor_value[6] = {0};
    long weighted_sum = 0, sum = 0;
#ifdef SERIAL_DEBUG
    Serial.print("   Sensor value:");
#endif
    for (int i = 0; i < 6; ++i) {
#ifndef DIGITAL
        //Using Digital Signal
        sensor_value[i] = analogRead(sensor_pin[i]);
#else
        //Using Analog Signal
        sensor_value[i] = analogRead(sensor_pin[i]);
#ifdef SERIAL_DEBUG
        Serial.print(' ');
        Serial.print(sensor_value[i]);
#endif
        //Double trigger line
        if (sensor_value[i] > calibration[i][1] && !pre_sensor_value[i] || 
            sensor_value[i] < calibration[i][0] && pre_sensor_value[i])
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
    //Bang bang control
    if (!sum)
        error = (abs(error_last) >= 5000)? constrain(error_last*3/2, -10000, 10000): error_last;
#ifdef SERIAL_DEBUG
    Serial.print("   Sum: ");
    Serial.print(sum);
    Serial.print("   Error: ");
    Serial.print(error);
#endif
    //PID
    int steering_cmd = pid();
#ifdef SERIAL_DEBUG
    Serial.print("  Steering cmd: ");
    Serial.print(steering_cmd);
#endif
    //write cmd
    steering.write(steering_cmd); 
}

void skip_barrier() {
    steering.write(STEERING_MED+STEERING_MAX);
    if(millis()-start_time > BARRIER_DELAY) state = normal;
}

int pid() {
    error_sum = error_sum*9/10 + error;
    long error_p = error;
    long error_d = error - error_last;
    long error_i = error_sum;
    //Speed boosting
#ifdef BOOSTING
    counter = (error_d)? 0: counter+1;
    if (counter >= 200)
        brushless.write(SPEED+10);
    else
        brushless.write(SPEED);
#endif
    //Caculate Error
    int steering_cmd = STEERING_MED + error_p * kp + error_i * ki + error_d * kd;
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

int ultrasonic() {
    long dura, dist;
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    dura = pulseIn(ECHO_PIN, HIGH, 5000); //timeout shouldn't be too long for blocking delay
    dist = (dura/2) / 29.1;
    return dist;
}

int get_dist() {
    int sum = 0, count = 0;
    int n = 5;
    for (int i = 0; i < n-1; ++i) dist_history[i] = dist_history[i+1];
    dist_history[n-1] = sonar.ping_cm();
    for (int i = 0; i < n; ++i) {
        if (dist_history[i])
            ++count;
        sum += dist_history[i];
    }
    if (!count) sum = 100;
    else sum /= count;
#ifdef SERIAL_DEBUG
    Serial.print("Dist: ");
    Serial.print(sum);   
#endif
    return sum;
}
