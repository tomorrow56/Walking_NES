#include <M5StickC.h>

#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x72);
#define SDA 0
#define SCL 26

//#define chgDET
#define DEBUG

/********************
 * ESPNow settings
 ********************/
esp_now_peer_info_t slave;
int modeNES = 3;  //0:Multicast, 1:M5SpiderW, 2:M5SpiderB, 3:FamiconSpider, other: Multicast
esp_err_t result;
boolean sending = false;

// command from Master
#define rNES_STOP 0
#define rNES_A 1
#define rNES_B 2
#define rNES_SELECT 3
#define rNES_START 4
#define rNES_UP 5
#define rNES_DOWN 6
#define rNES_LEFT 7
#define rNES_RIGHT 8

/********************************
* for Servo control
*********************************/
const uint8_t motorInterval = 100;  // default 30[ms]

// Published values for SG90 servos; 0-180 deg
int minUs = 500;
int maxUs = 2400;

// Servo assign to each channel
#define FR0 0   // knees
#define FL0 2
#define RR0 6
#define RL0 8

#define FR1 1   // leg
#define FL1 3
#define RR1 7
#define RL1 9

#define TILT 4

// center position calibration(0-180 deg)
#define OFST 20

uint8_t CENT_FR0 = 90 + OFST;   // knees UP ++
uint8_t CENT_FL0 = 90 - OFST;   // knees UP --
uint8_t CENT_RR0 = 90 - OFST;   // knees UP --
uint8_t CENT_RL0 = 90 + OFST;   // knees UP ++

uint8_t CENT_FR1 = 90;   // leg forward ++
uint8_t CENT_FL1 = 90;   // leg forward --
uint8_t CENT_RR1 = 90;   // leg backword ++
uint8_t CENT_RL1 = 90;   // leg backword --

uint8_t CENT_TILT = 90; // up --

#define ROTATEIN  40
#define ROTATEOUT  150

int stp = 3;
int smooth = 10;

int stepInterval = 50;
int stepIntervalTurn = 100;

boolean moveFlag = false;
boolean walkFwdFlag = false;
boolean walkBckFlag = false;
boolean walkLeftFlag = false;
boolean walkRightFlag = false;
boolean spinLeftFlag = false;
boolean spinRightFlag = false;

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: %s\n", macStr);
  Serial.printf("Last Packet Recv Data(%d): ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");

// 送信元でフィルタ
  if((String)macStr == "4C:75:25:C6:F8:0C" || (String)macStr == "D8:A0:1D:5B:D1:AC" || (String)macStr == "50:02:91:90:08:C0"){
    if(data[0] == rNES_A){
      Serial.println("NES_A");
      spinRightFlag = true;
      #ifdef DEBUG
        Serial.println("Spin Right");
      #endif
    }else if(data[0] == rNES_B){
      Serial.println("NES_B");
      spinLeftFlag = true;
      #ifdef DEBUG
        Serial.println("Spin Left");
      #endif
    }else if(data[0] == rNES_SELECT){
      Serial.println("NES_SELECT");
    }else if(data[0] == rNES_START){
      Serial.println("NES_START");
    }else if(data[0] == rNES_UP){
      Serial.println("NES_UP");
      walkFwdFlag = true;
      #ifdef DEBUG
        Serial.println("Walk Forward");
      #endif
    }else if(data[0] == rNES_DOWN){
      Serial.println("NES_DOWN");
      walkBckFlag = true;
      #ifdef DEBUG
        Serial.println("Walk Backward");
      #endif
    }else if(data[0] == rNES_LEFT){
      Serial.println("NES_LEFT");
      walkLeftFlag = true;
      #ifdef DEBUG
        Serial.println("Walk Left");
      #endif
    }else if(data[0] == rNES_RIGHT){
      Serial.println("NES_RIGHT");
      walkRightFlag = true;
      #ifdef DEBUG
        Serial.println("Walk Right");
      #endif
    }else if(data[0] == rNES_STOP){
      Serial.println("NES_STOP");
      walkStop();
    }else{
      Serial.println("Unknown");
    }
  }
}

/************************************
 * Arduino block
 ************************************/
void setup() {
  // M5Stack::begin(LCDEnable, PowerEnable, SerialEnable);
  M5.begin(true, true, true);
  Wire.begin(SDA, SCL);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);

  Serial.println("M5StickC Famicon Spider esp-now");
  Serial.printf("NES mode: %d\n", modeNES);
  M5.Lcd.println("NES Spider");
  M5.Lcd.printf("NES mode: %d\n", modeNES);

/*
 * Servo setup
 */
 //modeNES 1:M5SpiderW, 2:M5SpiderB, 3:Famicon Spider
  if(modeNES == 1){
    CENT_FR0 = 90 + OFST;   // knees UP ++
    CENT_FL0 = 77 - OFST;   // knees UP --
    CENT_RR0 = 90 - OFST;   // knees UP --
    CENT_RL0 = 100 + OFST;   // knees UP ++
    CENT_FR1 = 80;   // leg forward ++
    CENT_FL1 = 95;   // leg forward --
    CENT_RR1 = 100;   // leg backword ++
    CENT_RL1 = 82;   // leg backword --
    CENT_TILT = 100; // up --
  }else if(modeNES == 2){
    CENT_FR0 = 85 + OFST;   // knees UP ++
    CENT_FL0 = 85 - OFST;   // knees UP --
    CENT_RR0 = 95 - OFST;   // knees UP --
    CENT_RL0 = 90 + OFST;   // knees UP ++
    CENT_FR1 = 85;   // leg forward ++
    CENT_FL1 = 90;   // leg forward --
    CENT_RR1 = 90;   // leg backword ++
    CENT_RL1 = 95;   // leg backword --
    CENT_TILT = 110; // up --
  }else if(modeNES == 3){
    CENT_FR0 = 85 + OFST;   // knees UP ++
    CENT_FL0 = 90 - OFST;   // knees UP --
    CENT_RR0 = 85 - OFST;   // knees UP --
    CENT_RL0 = 85 + OFST;   // knees UP ++
    CENT_FR1 = 85;   // leg forward ++
    CENT_FL1 = 95;   // leg forward --
    CENT_RR1 = 85;   // leg backword --
    CENT_RL1 = 97;   // leg backword ++
    CENT_TILT = 110; // up --
  }

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at 50 Hz updates
  delay(10);

  Serial.println("Ready");
  M5.Lcd.print("Ready!");

/*
 * Demo action
 */
//  servoDemo();
  servoHome();

  #ifdef chgDET
    if(getChargeEnable()){
      adjMode();
    }
  #endif

  walkStop();

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

// ESP-NOWコールバック登録
  esp_now_register_recv_cb(OnDataRecv);

  // xTaskCreatePinnedToCore(タスクの関数名,"タスク名",スタックメモリサイズ,NULL,タスク優先順位,タスクハンドルポインタ,Core ID);
  // xTaskCreatePinnedToCore(func,"name",Stuck size,NULL,priority,Task pointer,Core ID)
  // Core ID: 0 or 1 or tskNO_AFFINITY
  // xTaskCreatePinnedToCore(avatorUpdate, "Task0", 4096, NULL, 1, NULL, 0);
}

void loop(){
  M5.update();

  if(walkFwdFlag == true){
    if(moveFlag == false){
      walkFwdInit();
      moveFlag = true;
    }
    stepFoward();
  }

  if(walkBckFlag == true){
    if(moveFlag == false){
      walkBckInit();
      moveFlag = true;
    }
    stepBackward();
  }

  if(walkLeftFlag == true){
    if(moveFlag == false){
      walkLeftInit();
      moveFlag = true;
    }
    stepLeft();
  }

  if(walkRightFlag == true){
    if(moveFlag == false){
      walkRightInit();
      moveFlag = true;
    }
    stepRight();
  }

  if(spinLeftFlag == true){
    if(moveFlag == false){
      spinLeftInit();
      moveFlag = true;
    }
    stepSpinLeft();
  }

  if(spinRightFlag == true){
    if(moveFlag == false){
      spinRightInit();
      moveFlag = true;
    }
    stepSpinRight();
  }

}

/************************************
 * Demo Action
 ************************************/
// Servo Demo
void servoDemo(){
  servoHome();
  delay(500);

  servoLegOpen(45);
  delay(500);
  servoStandUp(70);
  delay(500);
  servoLegClose(45);
  delay(500);
  servoLayDown(70);
  delay(500);

  walkStop();
  delay(500);

  servoTilt(10);
  delay(500);
  servoTilt(0);
}

// Home position
void servoHome(){
  moveServo(TILT, 0);

  moveServo(FR1, 0);
  moveServo(FL1, 0);
  moveServo(RR1, 0);
  moveServo(RL1, 0);

  moveServo(FR0, 0);
  moveServo(FL0, 0);
  moveServo(RR0, 0);
  moveServo(RL0, 0);
}

/************************************
 * Stop
 ************************************/
void walkStop(){
  walkFwdFlag = false;
  walkBckFlag = false;
  walkLeftFlag = false;
  walkRightFlag = false;
  spinLeftFlag = false;
  spinRightFlag = false;

  moveFlag = false;
  walkEnd();

  #ifdef DEBUG
    Serial.println("Walk Stop");
  #endif
}

// END position
void walkEnd(){
  delay(stepInterval);
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(motorInterval);
}

/************************************
 * Walk
 ************************************/
// Walk Forward
void walkFwdInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepInterval);

  // initial position for walk Forward
  moveServo(RL0, -50);
  delay(motorInterval);
  moveServo(RL1, 0);
  delay(stepInterval);
  moveServo(RL0, -70);
  delay(motorInterval);
}

void stepFoward(){
  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(stepInterval);

  moveServo(FL1, 45);
  delay(stepInterval);

  moveServo(FR1, 0);
  delay(motorInterval);
  moveServo(RL1, 45);
  delay(stepInterval);

  moveServo(RR1, 0);
  delay(stepInterval);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepInterval);

  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(stepInterval);

  moveServo(FR1, 45);
  delay(stepInterval);

  moveServo(FL1, 0);
  delay(motorInterval);
  moveServo(RR1, 45);
  delay(stepInterval);

  moveServo(RL1, 0);
  delay(stepInterval);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepInterval);
}

// Walk Backward
void walkBckInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepInterval);

  // initial position for walk Back
  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(FR1, 0);
  delay(stepInterval);
  moveServo(FR0, -70);
  delay(motorInterval);
}

void stepBackward(){
  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(stepInterval);

  moveServo(RR1, 45);
  delay(stepInterval);

  moveServo(FR1, 45);
  delay(motorInterval);
  moveServo(RL1, 0);
  delay(stepInterval);

  moveServo(FL1, 0);
  delay(stepInterval);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepInterval);

  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(stepInterval);

  moveServo(RL1, 45);
  delay(stepInterval);

  moveServo(FL1, 45);
  delay(motorInterval);
  moveServo(RR1, 0);
  delay(stepInterval);

  moveServo(FR1, 0);
  delay(stepInterval);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepInterval);
}

// Walk Left
void walkLeftInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepInterval);

  // initial position for walk Left
  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(FR1, 90);
  delay(stepInterval);
  moveServo(FR0, -70);
  delay(motorInterval);
}

void stepLeft(){
  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(stepInterval);

  moveServo(FL1, 45);
  delay(stepInterval);

  moveServo(FR1, 45);
  delay(motorInterval);
  moveServo(RL1, 90);
  delay(stepInterval);

  moveServo(RR1, 90);
  delay(stepInterval);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepInterval);

  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(stepInterval);

  moveServo(RL1, 45);
  delay(stepInterval);

  moveServo(FL1, 90);
  delay(motorInterval);
  moveServo(RR1, 45);
  delay(stepInterval);

  moveServo(FR1, 90);
  delay(stepInterval);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepInterval);
}

// Walk Right
void walkRightInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepInterval);

  // initial position for walk Right
  moveServo(RL0, -50);
  delay(motorInterval);
  moveServo(RL1, 90);
  delay(stepInterval);
  moveServo(RL0, -70);
  delay(motorInterval);
}

void stepRight(){
  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(stepInterval);

  moveServo(RR1, 45);
  delay(stepInterval);

  moveServo(RL1, 45);
  delay(motorInterval);
  moveServo(FR1, 90);
  delay(stepInterval);

  moveServo(FL1, 90);
  delay(stepInterval);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepInterval);

  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(stepInterval);

  moveServo(FR1, 45);
  delay(stepInterval);

  moveServo(FL1, 45);
  delay(motorInterval);
  moveServo(RR1, 90);
  delay(stepInterval);

  moveServo(RL1, 90);
  delay(stepInterval);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepInterval);
}

/************************************
 * Spin
 ************************************/
// Spin Left
void spinLeftInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepIntervalTurn);
}

void stepSpinLeft(){
  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(motorInterval);
  moveServo(FR1, 90);
  delay(motorInterval);
  moveServo(RL1, 90);
  delay(motorInterval);
  moveServo(FL1, 90);
  delay(motorInterval);
  moveServo(RR1, 90);
  delay(stepIntervalTurn);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepIntervalTurn);

  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(motorInterval);
  moveServo(FL1, 45);
  delay(motorInterval);
  moveServo(RR1, 45);
  delay(motorInterval);
  moveServo(FR1, 45);
  delay(motorInterval);
  moveServo(RL1, 45);
  delay(stepIntervalTurn);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepIntervalTurn);
}

// Spin Right
void spinRightInit(){
  servoLegFront(45);
  delay(motorInterval);
  servoLegRear(45);
  delay(motorInterval);
  servoKneesAll(70);
  delay(stepIntervalTurn);
}

void stepSpinRight(){
  moveServo(FL0, -50);
  delay(motorInterval);
  moveServo(RR0, -50);
  delay(motorInterval);
  moveServo(FR1, 90);
  delay(motorInterval);
  moveServo(RL1, 90);
  delay(motorInterval);
  moveServo(FL1, 90);
  delay(motorInterval);
  moveServo(RR1, 90);
  delay(stepIntervalTurn);

  moveServo(FL0, -70);
  delay(motorInterval);
  moveServo(RR0, -70);
  delay(stepIntervalTurn);

  moveServo(FR0, -50);
  delay(motorInterval);
  moveServo(RL0, -50);
  delay(motorInterval);
  moveServo(FL1, 45);
  delay(motorInterval);
  moveServo(RR1, 45);
  delay(motorInterval);
  moveServo(FR1, 45);
  delay(motorInterval);
  moveServo(RL1, 45);
  delay(stepIntervalTurn);

  moveServo(FR0, -70);
  delay(motorInterval);
  moveServo(RL0, -70);
  delay(stepIntervalTurn);
}

/************************************
 * fixed action
 ************************************/
// all knees angle
void servoKneesAll(int ang){
  int i = (-1 * ang);
  moveServo(FR0, i);
  delay(motorInterval);
  moveServo(FL0, i);
  delay(motorInterval);
  moveServo(RR0, i);
  delay(motorInterval);
  moveServo(RL0, i);
  delay(motorInterval);
}

// front leg angle
void servoLegFront(int ang){
  int i = ang;
  moveServo(FR1, i);
  delay(motorInterval);
  moveServo(FL1, i);
  delay(motorInterval);
}

// Rear leg angle
void servoLegRear(int ang){
  int i = ang;
  moveServo(RR1, i);
  delay(motorInterval);
  moveServo(RL1, i);
  delay(motorInterval);
}

// Stand Up
void servoStandUp(int ang){
  int i = 0;
  while(i < ang){
    servoKneesAll(i);
    i = i + stp;
    delay(smooth);
  }
}

// Lay Down
void servoLayDown(int ang){
  // lay down
  int i = ang;
  while(i > 0){
    servoKneesAll(i);
    i = i - stp;
    delay(smooth);
  }
}

// Leg Open
void servoLegOpen(int ang){
  int i = 0;
  while(i < ang){
    servoLegFront(i);
    delay(motorInterval);
    servoLegRear(i);
    i = i + stp;
    delay(smooth);
  }
}

// Leg Close
void servoLegClose(int ang){
  int i = ang;
  // leg close
  while(i > 0){
    servoLegFront(i);
    delay(motorInterval);
    servoLegRear(i);
    i = i - stp;
    delay(smooth);
  }
}

// Tilt
void servoTilt(int ang){
  moveServo(TILT, ang);
}

/************************************
 * Servo position by angle
 ************************************/
void moveServo(int n, int deltaAngle){
  if(n == TILT){
     setServoAngle(n, CENT_TILT - deltaAngle);
     delay(motorInterval);
   }else if(n == FR1){
     setServoAngle(n, CENT_FR1 + deltaAngle);
     delay(motorInterval);
   }else if(n == FL1){
     setServoAngle(n, CENT_FL1 - deltaAngle);
     delay(motorInterval);
   }else if(n == RR1){
     setServoAngle(n, CENT_RR1 - deltaAngle);
     delay(motorInterval);
   }else if(n == RL1){
     setServoAngle(n, CENT_RL1 + deltaAngle);
     delay(motorInterval);
   }else if(n == FR0){
     setServoAngle(n, CENT_FR0 + deltaAngle);
    delay(motorInterval);
   }else if(n == FL0){
     setServoAngle(n, CENT_FL0 - deltaAngle);
     delay(motorInterval);
   }else if(n == RR0){
     setServoAngle(n, CENT_RR0 - deltaAngle);
     delay(motorInterval);
   }else if(n == RL0){
     setServoAngle(n, CENT_RL0 + deltaAngle);
     delay(motorInterval);
   }
}

/*************************************************** 
 you can use this function if you'd like to set the pulse length in microsecond
***************************************************/
void setServoPulse(uint8_t n, double pulse_us){
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // 50 Hz
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse_us /= pulselength;
//  Serial.println(pulse_us);

  pwm.setPWM(n, 0, pulse_us);
}

/*************************************************** 
 you can use this function if you'd like to set the angle in degree
***************************************************/
void setServoAngle(uint8_t n, uint8_t ang){
  double pulse_us = map(ang, 0, 180, minUs, maxUs); //map angle(0～180) to pulse_us
  setServoPulse(n, pulse_us);
}

/*************************************************** 
 status of charging
 **************************************************/
boolean getChargeEnable(){
  boolean result = false;
  float vbus = M5.Axp.GetVBusVoltage();
  Serial.println((String)vbus);
  M5.Lcd.setCursor(10, 25);
  M5.Lcd.print("VBUS: " + (String)vbus + "V");
  delay(1000);
  if(vbus >= 2){
    result = true;
  }
  return(result);
}
void adjMode(){
    delay(300);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("HOME POSITION");
    servoHome();

    #ifdef DEBUG
      Serial.println("Charging...");
    #endif

    while(1){
      M5.update();
      M5.Lcd.setCursor(10, 50);
      float Level =  M5.Axp.GetBatVoltage();
      M5.Lcd.print("BAT: " + (String)Level + "V");

      if(M5.BtnA.wasPressed()){
        M5.Lcd.fillScreen(BLACK);
        break;
      }
      delay(100);
   }
}
