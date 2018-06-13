/************************************************
    FileName    [ formula_air_tracking.ino ]
    Synopsis    [ Tracking line using PID and bang bang control ]
    Author      [ Johnson Shih, Frank Lin ]
    Copyleft    [ Group 15 小萌牛©2018, NTUME ]
************************************************/

/********************Options********************/
#define SERIAL_DEBUG
/********************Include********************/
#include <PS2X_lib.h>
#include <Servo.h>
/********************Pins***********************/
#define PS2_DAT_PIN A3
#define PS2_CMD_PIN A2
#define PS2_SEL_PIN A1
#define PS2_CLK_PIN A0
#define BLUSHLESS_PIN  9
#define STEERING_PIN   6
/********************Other Parameters***********/
#define STEERING_MAX 35
#define STEERING_MED 90
#define SPEED_IDLE 15
#define SPEED_MAX 60
#define SPEED 30 
#define SPEED_LOW 25 
#define SPEED_MED 35
#define SPEED_HIGH 45
#define SPEED_DELTA 2

/********************Objects********************/
Servo brushless;
Servo steering;
PS2X ps2x;
/********************Variables******************/
enum Status {disconn, conn};
Status state = disconn;
int brushless_cmd = SPEED_IDLE;
int speed_low = SPEED_LOW;
int speed_med = SPEED_MED;
int speed_high = SPEED_HIGH;
int speed = SPEED_IDLE;
int vibrate_count = 0;
int vibrate_amp = 50;

void ps2x_cmd();
void ps2x_conn();
void ps2x_button();
void brushless_init();
int nonlinearMap(int);
void vibrate(int, int = 10);

void setup() {
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    brushless.attach(BLUSHLESS_PIN);
    steering.attach(STEERING_PIN);
    steering.write(STEERING_MED);
    state = disconn;
    brushless_init();
}

void loop() {
    if (state==disconn)
        ps2x_conn();
    else
        ps2x_cmd();
    delay(1);
}

void ps2x_conn() {
#ifdef SERIAL_DEBUG
    Serial.println("Connecting to gameoad...");
#endif
    int error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, false, false);
    if (!error) {
        state = conn;
#ifdef SERIAL_DEBUG
        Serial.println("Connect Successfully");
#endif
    }
    else
#ifdef SERIAL_DEBUG
        Serial.println("Cannot connect to gamepad, retrying...");
#endif
    delay(1000);
}

void ps2x_cmd() {
    int steering_cmd = 0;
    ps2x.read_gamepad(false, false);
    //if (vibrate_count) --vibrate_count;

    int ly = ps2x.Analog(PSS_LY);
    int rx = ps2x.Analog(PSS_RX);
    int ry = ps2x.Analog(PSS_RY);
    int lx = ps2x.Analog(PSS_LX);
    if (ly == 255 && lx == 255) return;

    steering_cmd = nonlinearMap(lx);
    ps2x_button();
         
#ifdef SERIAL_DEBUG
    Serial.print(" Steering_cmd: ");
    Serial.print(steering_cmd);
    Serial.print(" Brushless_cmd: ");
    Serial.print(brushless_cmd);
#endif
    brushless.write(brushless_cmd);
    steering.write(steering_cmd);

#ifdef SERIAL_DEBUG
    Serial.println("");
#endif
}

/************************************************
PSB_START, PSB_SELECT, PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_LEFT, PSB_PAD_DOWN
PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
PSB_L1, PSB_R1, PSB_L2, PSB_R2, PSB_L3, PSB_R3
************************************************/
bool preUP = false, preDOWN = false, preLEFT = false, preRIGHT = false;
bool preSQ = false, preCRO = false, preTRI = false, preCIR = false;
bool preR1 = false, preR2 = false, preL1 = false, preL2 = false;
void ps2x_button() {
    if (ps2x.NewButtonState()) {
        bool currUP = ps2x.Button(PSB_PAD_UP), currDOWN = ps2x.Button(PSB_PAD_DOWN), currLEFT = ps2x.Button(PSB_PAD_LEFT), currRIGHT = ps2x.Button(PSB_PAD_RIGHT);
        bool currSQ = ps2x.Button(PSB_SQUARE), currCRO = ps2x.Button(PSB_CROSS), currTRI = ps2x.Button(PSB_TRIANGLE), currCIR = ps2x.Button(PSB_CIRCLE);
        bool currR1 = ps2x.Button(PSB_R1), currR2 = ps2x.Button(PSB_R2), currL1 = ps2x.Button(PSB_L1), currL2 = ps2x.Button(PSB_L2);
        if (currUP && !preUP) {
            if (currTRI) speed_high += SPEED_DELTA;
            else if (currCIR) speed_med += SPEED_DELTA;
            else if (currCRO) speed_low += SPEED_DELTA;
        } 
        if (currDOWN && !preDOWN) {
            if (currTRI) speed_high -= SPEED_DELTA;
            else if (currCIR) speed_med -= SPEED_DELTA;
            else if (currCRO) speed_low -= SPEED_DELTA;
        }
        if (currSQ&& !preSQ) {
            speed_low = SPEED_LOW;
            speed_med = SPEED_MED;
            speed_high = SPEED_HIGH;
        }

        //vibrate(50);
        speed = constrain(speed, SPEED_IDLE, SPEED_MAX);

        brushless_cmd = currTRI? speed_high: currCIR? speed_med: currCRO? speed_low: SPEED_IDLE;

        preUP = currUP, preDOWN = currDOWN, preLEFT = currLEFT, preRIGHT = currRIGHT;
        preSQ = currSQ, preCRO = currCRO, preTRI = currTRI, preCIR = currCIR;
        preR1 = currR1, preR2 = currR2, preL1 = currL1, preL2 = currL2;
    }
}

void brushless_init() {
    brushless.write(100);
    delay(50);
    brushless.write(SPEED_IDLE);
    delay(3000);
}

int nonlinearMap(int x) {
    if (x == 0) return STEERING_MED-STEERING_MAX;
    if (x == 255) return STEERING_MED+STEERING_MAX;
    return map(x, 0, 255, STEERING_MED-STEERING_MAX+10, STEERING_MED+STEERING_MAX-10);
}

void vibrate(int n = 100, int amp = 10) {
    vibrate_count = n;
    vibrate_amp = amp;
}
