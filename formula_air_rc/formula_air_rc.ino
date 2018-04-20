#include <PS2X_lib.h>
#include<Servo.h>

#define PS2_DAT        2
#define PS2_CMD        3
#define PS2_SEL        4
#define PS2_CLK        5
#define BLUSHLESS_PIN  9
#define STEERING_PIN   6

#define pressures   false
#define rumble      false

Servo brushless;
Servo steering;
PS2X ps2x;

int error = 1;
int brushless_cmd = 0;

void ps2x_cmd();
void brushless_init();

void setup() {
  Serial.begin(9600);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  digitalWrite(13, error);

  brushless.attach(BLUSHLESS_PIN);
  brushless_init();
  steering.attach(STEERING_PIN);
  steering.write(90);
}

void loop() {
  
  ps2x_cmd();
}

//PSB_START, PSB_SELECT, PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_LEFT, PSB_PAD_DOWN
//PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
//PSB_L1, PSB_R1, PSB_L2, PSB_R2, PSB_L3, PSB_R3

int preRX = 127, preLY = 127;
bool preCmd;
int delta = 70;
void ps2x_cmd() {
  if(error) return;
  
  ps2x.read_gamepad(false, 0);
  
  if (ps2x.NewButtonState()) {
    /*
    if(ps2x.ButtonPressed(PSB_L1))
      led_l = !led_l;
    else if(ps2x.ButtonPressed(PSB_R1))
      led_r = !led_r;

    if(ps2x.ButtonPressed(PSB_L2))
      dir_l = !dir_l;
    else if(ps2x.ButtonPressed(PSB_R2))
      dir_r = !dir_r;
    
    if(ps2x.Button(PSB_SQUARE)) Serial.println("1");
    else Serial.println("0");
    */
    if(ps2x.Button(PSB_SQUARE)) {
      if(brushless_cmd <= 170 && !preCmd)
        brushless_cmd += 10;
      preCmd = true;
    }
      
    else if(ps2x.Button(PSB_CROSS)) {
      if(brushless_cmd >= 25 && !preCmd)
        brushless_cmd -= 10;
      preCmd = true;
    }
    else
      preCmd = false;
  }
  int ly = ps2x.Analog(PSS_LY);
  int rx = ps2x.Analog(PSS_RX);

  if (rx==255 && preRX<=130 || rx==0 && preRX>=125) rx = preRX;
  if (ly==255 && preLY<=130 || ly==0 && preLY>=125) ly = preLY;
//  Serial.println(brushless_cmd);
  if(ly < 88)
    brushless.write(brushless_cmd);
  else
    brushless.write(15);
  
  steering.write(map(rx, 0, 255, 90-delta, 90+delta));

  preRX = rx;
  preLY = ly;
}

void brushless_init() {
  brushless_cmd = 180;
  brushless.write(brushless_cmd);
  delay(100);
  brushless_cmd = 15;
  brushless.write(brushless_cmd);
  delay(500);
}
