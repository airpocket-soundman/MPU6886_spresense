//pop と　appendの関数できた。244～274usecなので333usecで割り込みして処理。
//https://ja.stackoverflow.com/questions/51916/spresense-arduino-ide-%E3%82%BF%E3%82%A4%E3%83%9E%E3%83%BC%E5%89%B2%E8%BE%BC%E3%81%BF
//これのHPを参考に割り込みでIMUを読む関数をつくって、データ配列を常に更新する。
//加速度が閾値を超えたら、そこから規定数のデータを採取して保存する。

#include <Wire.h>
#include "LGFX_SPRESENSE_sample.hpp"
#include "MPU6886.h"
#include <Flash.h>
#include <SDHCI.h>

#define SAMPLE_SIZE   320       //加速データのサンプル採取数
#define PLAY_BACK     50        //閾値以上の加速度を検知してからさかのぼって記録するサンプル数
#define SAMPLE_CYCLE  333       //サンプリング周期

#define X0 5                  // 横軸の描画開始座標
#define MINZ -1000            // 縦軸の最小値 -16000 or -1
#define MAXZ 1000             // 縦軸の最大値  16000 or  1 

const int button4 = 4;
const int button5 = 5;
const int button6 = 6;
const int button7 = 7;
int label = 0;
int labelNum = 10;

static long timeLog[SAMPLE_SIZE];    //サンプリング周期記録用
static long startTime;               //サンプリング開始時間
static long finishTime;              //サンプリング終了時間
static long timeNow;
float ax[SAMPLE_SIZE], ay[SAMPLE_SIZE], az[SAMPLE_SIZE];    // 加速度データを読み出す変数
float azN[SAMPLE_SIZE];
static unsigned long counter = PLAY_BACK;     //
unsigned int delayTime = 100;
int number = 0;                               //採取したデータの通し番号

boolean serialOut  = false;                    //シリアル出力する場合はtrue
boolean graphOut   = true;                    //加速度をグラフ出力する場合はtrue
boolean fileOut    = true;
static LGFX lcd;
MPU6886 IMU;

void output(float* az, long* timeLog);                //アウトプット制御関数
void serialOutput(int i, float* az, long* timeLog);   //シリアル出力する関数
void drawGraphLine(int i, float* az);                 //加速度をグラフ出力する関数
unsigned int popAppend();                             //加速度データをリストに記録する関数
void normalization(float* az);
void saveFile();
void pushed4();
void pushed5();
void pushed6();
void pushed7();


void setup() {

//display setup
  lcd.init();
  lcd.fillScreen(TFT_BLACK);
  lcd.setCursor(0,0);
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setRotation(1);

//button setup
  pinMode(button4, INPUT_PULLUP);
  pinMode(button5, INPUT_PULLUP);
  pinMode(button6, INPUT_PULLUP);
  pinMode(button7, INPUT_PULLUP);
  attachInterrupt(button4, pushed4, FALLING);
  attachInterrupt(button5, pushed5, FALLING);
  attachInterrupt(button6, pushed6, FALLING);
  attachInterrupt(button7, pushed7, FALLING);
  
//serial setup
  Wire.begin();
  Wire.setClock(500000);
  Serial.begin(115200);
  Serial.println("start");

//IMU setup
  IMU.Init();
  timeLog[SAMPLE_SIZE - 1] = micros();
}

void loop() {
//  int i = 0;
//  readButtonState();
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
    lcd.setCursor(120,0);
    lcd.print("label:");
    lcd.print(label);
    lcd.print(" num:");
    lcd.println(number);
    normalization(az);
    output(azN, timeLog);
  }
}

void pushed4(){
  lcd.setCursor(120,0);
  label += 1;
  if (label == 10) label = 0;
  lcd.print("label:");
  lcd.print(label);
  lcd.print(" num:");
  lcd.println(number);
  lcd.setCursor(0,200);
  lcd.println("4");
}

void pushed5(){
  lcd.setCursor(0,200);
  lcd.println("5");
}

void pushed6(){
  lcd.setCursor(0,200);
  lcd.println("6");
}

//number reset & flash mem clear
void pushed7(){
  lcd.setCursor(0,200);
  lcd.println("7");
  number = 0;         //sample 通しナンバーリセット
/*  
  File flash_root = Flash.open("/");
  while (true){
    File from = flash_root.openNextFile();
    if (!from) break;
    if (from.isDirectory()) continue;
    String str(from.name());
    str = str.substring(11);
    if (!str.endsWith(".csv")) continue;
    Serial.println(str);
    Flash.remove(str);
  }
  Serial.println("all files erased.");
*/
}

void saveData(){
  char fname[16];
  sprintf(fname, "%01d%03d.csv", label, number);
  if (Flash.exists(fname)) Flash.remove(fname);
  File myFile = Flash.open(fname,FILE_WRITE);
  if (!myFile){
    Serial.println("File Open Error: "+String(fname));
    return;
  }
  for (int i = 0; i<SAMPLE_SIZE; i++){
    myFile.println(String(azN[i],6));
    Serial.println(String(azN[i],6));
  }
  myFile.close();
  number++;
}

void normalization(float* az){
  float azMax = az[0];
  float azMin = az[0];
  for (int i = 0; i <SAMPLE_SIZE; i++){
    if (az[i] > azMax) azMax = az[i];
    if (az[i] < azMin) azMin = az[i];
  }
  float range = abs(azMax) > abs(azMin) ? abs(azMax) : abs(azMin);  //三項演算子
  for (int i = 0; i < SAMPLE_SIZE; i++){
    azN[i] = az[i]/range;
  }
}

void drawGraphLine(int i, float* az){
  if (i > 1) {
      int y0 = map(az[i - 1]*1000, MINZ, MAXZ, lcd.height(), 0);
      int y1 = map(az[i]*1000, MINZ, MAXZ, lcd.height(), 0);
      lcd.drawLine(i - 1 + X0, y0, i + X0, y1, TFT_GREEN);
  }
}

void serialOutput(int i, float* az, long* timeLog){
  Serial.print("cycle(usec):");
  Serial.print(timeLog[i + 1] - timeLog[i]);
  Serial.print(" , Zaxis accelerration(G):");
  Serial.print(az[i]);
  Serial.print(" , sample num:");
  Serial.println(i); 
}

void output(float* azN, long* timeLog){
  for (int i = 0; i < SAMPLE_SIZE; i++){
    if (serialOut == true) serialOutput(i, azN, timeLog);
    if (graphOut == true) drawGraphLine(i, azN); 
  }
  if (fileOut == true) saveData();
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
