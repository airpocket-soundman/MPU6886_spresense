#include <Wire.h>
#include <LGFX_SPRESENSE_sample.hpp>
#include "MPU6886.h"

#define SAMPLE_SIZE   320       //
#define PLAY_BACK     50
#define SAMPLE_CYCLE  333    

#define X0 5                  // 横軸の描画開始座標
#define MINZ -16000           // 縦軸の最小値
#define MAXZ 16000            // 縦軸の最大値

static unsigned long timeLog[SAMPLE_SIZE];    //サンプリング周期記録用
static unsigned long startTime;
static unsigned long finishTime;
static unsigned long timeNow;
float ax[SAMPLE_SIZE], ay[SAMPLE_SIZE], az[SAMPLE_SIZE];    // 加速度データを読み出す変数
static unsigned long counter = PLAY_BACK;
unsigned int delayTime = 100;

boolean serialOut  = true;
boolean graphOut   = true;

static LGFX lcd;
MPU6886 IMU;

void serialOutput(int i, float* az, long* timeLog);
void output(float* az, long* timeLog);
void drawGraphLine(int i, float* az);
unsigned int popAppend();
unsigned int test();

void setup() {

  lcd.init();
  lcd.fillScreen(TFT_BLACK);
  lcd.setCursor(0,0);
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setRotation(1);

  Wire.begin();
  Wire.setClock(500000);
  Serial.begin(115200);
  Serial.println("start");
  IMU.Init();
  timeLog[SAMPLE_SIZE - 1] = micros();
}

void loop() {
  int i = 0;

  popAppend();
  if (az[SAMPLE_SIZE - 1] > 1.5 && counter >= SAMPLE_SIZE) {
    counter = PLAY_BACK;
    startTime = micros();
  }
  if (counter == SAMPLE_SIZE && micros() > 3000000){
    finishTime = micros();
    lcd.fillScreen(TFT_BLACK);  // 画面をクリア
    lcd.setCursor(0, 0);
    lcd.println((finishTime - startTime)/(SAMPLE_SIZE - PLAY_BACK));
    output(az, timeLog);
  }
}

void drawGraphLine(int i, float* az){
  if (i > 1) {
      int y0 = map((int)(az[i - 1] * 1000), MINZ, MAXZ, lcd.height(), 0);
      int y1 = map((int)(az[i] * 1000), MINZ, MAXZ, lcd.height(), 0);
      lcd.drawLine(i - 1 + X0, y0, i + X0, y1, TFT_GREEN);
  }
}

void serialOutput(int i, float* az, long* timeLog){
  Serial.print("cycle(usec):");
  Serial.print(timeLog[i + 1] - timeLog[i]);
  Serial.print(" , Zaxis accelerration(mG):");
  Serial.print(az[i]);
  Serial.print(" , sample num:");
  Serial.println(i); 
}

void output(float* az, long* timeLog){
  for (int i = 0; i < SAMPLE_SIZE; i++){
    if (serialOut == true) serialOutput(i, az, timeLog);
    if (graphOut == true) drawGraphLine(i, az); 
  }
}

unsigned int test(){
//  timeNow = micros();
  counter++;
  float azNow = 0;
  int16_t azRaw;
  uint8_t buf[2];
 
  while ((micros() - timeLog[SAMPLE_SIZE - 1]) < SAMPLE_CYCLE - 10) delayMicroseconds(3);
  timeNow = micros();
  Wire.beginTransmission(0x68);
  Wire.write(0x3F);  
  Wire.endTransmission();
  uint8_t i = 0;
  Wire.requestFrom(0x68,2);
  
  //! Put read results in the Rx buffer
  while (Wire.available()) {
    buf[i++] = Wire.read();
  }
  azRaw = ((int16_t)buf[0]<<8)|buf[1];
  azNow = (float)azRaw * IMU.aRes;


  for (int i = 0; i < SAMPLE_SIZE-1; i++){
    az[i] = az[i + 1];
    timeLog[i] = timeLog[i + 1];
  }
  timeLog[SAMPLE_SIZE - 1] = timeNow;
  az[SAMPLE_SIZE -1] = azNow;
  counter++;
  return SAMPLE_CYCLE;
}

unsigned int popAppend(){
  float azNow = 0;
  while ((micros() - timeLog[SAMPLE_SIZE - 1]) < SAMPLE_CYCLE - 20) delayMicroseconds(2);
  timeNow = micros();
  IMU.getAccelZData(&azNow);
  for (int i = 0; i < SAMPLE_SIZE-1; i++){
    az[i] = az[i + 1];
    timeLog[i] = timeLog[i + 1];
  }
  timeLog[SAMPLE_SIZE - 1] = timeNow;
  az[SAMPLE_SIZE -1] = azNow;
  counter++;
  return SAMPLE_CYCLE;

}
