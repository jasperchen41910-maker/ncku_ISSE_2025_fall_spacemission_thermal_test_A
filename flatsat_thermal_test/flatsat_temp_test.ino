#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Adafruit_GPS.h>
#include <DFRobot_Geiger.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Arduino.h>
#include "Mcp320x.h"

// ================= IMU / MAG =================
SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

// ================= GPS =================
#define GPSSerial Serial5
#define LoRaSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

// ================= Quaternion =================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll_qua_deg = 0, pitch_qua_deg = 0, yaw_qua_deg = 0;
float bx = 0, by = 0, bz = 0;
unsigned long last_imu_micros = 0; // 計算 dt 用

// ================= Geiger counter =================
// 請確認你的硬體接線，程式碼設定為 Pin 22
DFRobot_Geiger geiger(22); 

// ================= DS18B20 =================
OneWire oneWire(2);
DallasTemperature sensors(&oneWire);

// ================= ADC =================
#define CS1 0
#define CLK 1000000
#define ADC_VREF 5000 
MCP3208 adc1(ADC_VREF, CS1, &SPI1);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  LoRaSerial.begin(9600);

  // ---- IMU / MAG ----
  if (!myISM.begin()) {
    Serial.println("6 DOF did not begin.");
    while (1);
  }
  myISM.deviceReset();
  while (!myISM.getDeviceReset()) delay(1);

  if (!myMag.begin()) {
    Serial.println("Magnetometer did not begin");
    while (1);
  }
  myMag.softReset();
  
  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();
  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_2g);
  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_2000dps);
  myISM.setAccelFilterLP2();
  myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
  myISM.setGyroFilterLP1();
  myISM.setGyroLP1Bandwidth(ISM_MEDIUM);

  Serial.println("IMU + MAG Ready");

  // ---- GPS ----
  GPSSerial.begin(19200);
  GPS.begin(19200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     
  GPS.sendCommand(PGCMD_ANTENNA);                 
  
  // ---- Geiger ----
  geiger.start();
  Serial.println("Geiger Ready (pin 22)");

  // ---- Temp ----
  sensors.begin();
  Serial.println("Temperature Ready");

  // ---- ADC ----
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPISettings settings(CLK, MSBFIRST, SPI_MODE0);
  SPI1.setMISO(1);
  SPI1.begin();
  SPI1.beginTransaction(settings);

  Serial.println("Packet2s,Time,Lat,Lon,Alt,Sat,Speed,Pitch,Roll,Yaw,q0,q1,q2,q3,bx,by,bz,timer,TempC");
  Serial.println("Packet10s,Time,Alt,CPM,uSvh,nSvh,counts,ADC0,ADC3,ADC4,ADC5,ADC6,ADCV");
  
  last_imu_micros = micros();
}

static uint32_t lastDataLog_1 = 0;  // 2 sec timer
static uint32_t lastDataLog_30 = 0; // 10 sec timer

void loop() 
{
  // 1. GPS 解析 (必須隨時執行)
  while (GPSSerial.available() > 0) {
    GPS.read();
  }
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // 2. IMU 運算 (必須隨時執行，不能放在 2秒 Timer 裡！)
  // 移出來才能積分正確
  if (myISM.checkStatus()) 
  {
    unsigned long now_micros = micros();
    float dt = (now_micros - last_imu_micros) / 1000000.0f;
    last_imu_micros = now_micros;

    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    // Magnetometer
    uint32_t x = myMag.getMeasurementX();
    uint32_t y = myMag.getMeasurementY();
    uint32_t z = myMag.getMeasurementZ();

    double sx = ((double)x - 133872) * 0.7277 / 131072.0;
    double sy = ((double)y - 131761) * 1.0 / 131072.0;
    double sz = -((double)z - 132287) * 1.1343 / 131072.0;

    bx = sx * 800;
    by = sy * 800;
    bz = sz * 800;

    // IMU Data
    float accelX_g = (float)accelData.xData / 989.0f;
    float accelY_g = (float)accelData.yData / 1009.48f;
    float accelZ_g = (float)accelData.zData / 1008.0f;
    float gyroX_dps = (float)gyroData.xData * 0.000872f - 0.183f;
    float gyroY_dps = (float)gyroData.yData * 0.000872f + 0.300f;
    float gyroZ_dps = 1.09f * ((float)gyroData.zData * 0.000872f - 0.061f);

    // Acc -> Pitch/Roll
    float total_g = sqrt(accelX_g*accelX_g + accelY_g*accelY_g + accelZ_g*accelZ_g);
    float roll_acc  = atan2(accelY_g, accelZ_g) * 180.0f / PI;
    float pitch_acc = asin(-accelX_g / total_g) * 180.0f / PI;

    // Gyro -> Quaternion
    float wx = gyroX_dps * PI / 180.0f;
    float wy = gyroY_dps * PI / 180.0f;
    float wz = gyroZ_dps * PI / 180.0f;

    float q0_dot = 0.5f * (-q1*wx - q2*wy - q3*wz);
    float q1_dot = 0.5f * ( q0*wx + q2*wz - q3*wy);
    float q2_dot = 0.5f * ( q0*wy - q1*wz + q3*wx);
    float q3_dot = 0.5f * ( q0*wz + q1*wy - q2*wx);

    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;

    float norm_q = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 0) {
      q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q;
    }

    // Qua -> Euler
    float pre_pitch = 2 * (q0*q2 - q1*q3);
    if (pre_pitch > 1) pre_pitch = 1;
    if (pre_pitch < -1) pre_pitch = -1;

    float pre_roll_1 = 2 * (q0*q1 + q2*q3);
    float pre_roll_2 = (1 - 2 * (q1*q1 + q2*q2));
    float pre_yaw_1 = (q0*q3 + q1*q2);
    float pre_yaw_2 = (1 - 2 * (q2*q2 + q3*q3));

    pitch_qua_deg = asin(pre_pitch) * 180.0f / PI;
    roll_qua_deg = atan2(pre_roll_1, pre_roll_2) * 180.0f / PI;
    yaw_qua_deg = atan2(pre_yaw_1, pre_yaw_2) * 180.0f / PI;

    if (abs(roll_acc - roll_qua_deg) > 5.0) roll_qua_deg = roll_acc;
    if (abs(pitch_acc - pitch_qua_deg) > 5.0) pitch_qua_deg = pitch_acc;
  }

  unsigned long currentTime = millis();

  // ============================ 2 sec loop (Print Only) ============================
  if (currentTime - lastDataLog_1 > 2000) 
  {
    lastDataLog_1 = currentTime;
    
    // 讀取溫度
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    float timer = lastDataLog_1 / 1000.0;
    Serial.print("Packet2s,");
    // GPS Print
    if (GPS.fix) {
      Serial.printf("%02d:%02d:%02d,", GPS.hour, GPS.minute, GPS.seconds);
      Serial.printf("%.5f,%.5f,%.1f,%d,%.2f,",
                    GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude,
                    (int)GPS.satellites, GPS.speed);
    } else {
      Serial.print("00:00:00,0.0,0.0,0.0,0,0.0,");
    }

    // Attitude Print
    Serial.printf("%.2f,%.2f,%.2f,", pitch_qua_deg, roll_qua_deg, yaw_qua_deg);
    Serial.printf("%.2f,%.2f,%.2f,%.2f,", q0, q1, q2, q3);
    
    // Magnetometer Print
    Serial.printf("%.2f,%.2f,%.2f,", bx, by, bz);
    
    // Temp Print (修正了這裡的 Crash 錯誤)
    Serial.printf("%.2f,%.2f", timer, tempC); 
    Serial.println();
  }

  // ============================= 10 sec loop (LoRa & Geiger) ==============================
  if (currentTime - lastDataLog_30 > 10000) 
  {
    lastDataLog_30 = currentTime;


     Serial.print("Packet10s,");
    if (GPS.fix) {
      Serial.printf("%02d:%02d:%02d,", GPS.hour, GPS.minute, GPS.seconds);
      Serial.printf("%.1f,", GPS.altitude);
    } else {
      Serial.print("00:00:00,0.0,");
    }
   
     // 讀取 Geiger 數據
    Serial.print(",");
    Serial.print(geiger.getCPM());
    Serial.print(",");
    Serial.print(geiger.getnSvh());
    Serial.print(",");
    Serial.print(geiger.getuSvh());
    Serial.print(",");
    Serial.print(geiger.getTotalCount());
    Serial.print(",");

    geiger.pause();
    // LoRa 發送
    LoRaSerial.print("AAAAAAAAAAA");//發送測試
    //恢復 Geiger 
    geiger.start();

    // ADC 讀取
    float raw_ch0 = adc1.read(MCP3208::Channel::SINGLE_0);
    float value_ch0 = adc1.toAnalog(raw_ch0);
    float raw_ch3 = adc1.read(MCP3208::Channel::SINGLE_3);
    float value_ch3 = adc1.toAnalog(raw_ch3);
    float raw_ch4 = adc1.read(MCP3208::Channel::SINGLE_4);
    float value_ch4 = adc1.toAnalog(raw_ch4);
    float raw_ch5 = adc1.read(MCP3208::Channel::SINGLE_5);
    float value_ch5 = adc1.toAnalog(raw_ch5);
    float raw_ch6 = adc1.read(MCP3208::Channel::SINGLE_6);
    float value_ch6 = adc1.toAnalog(raw_ch6);
    float actual_battery_v = value_ch0 - value_ch6;

    // ADC Print
    Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", value_ch0, value_ch3, value_ch4, value_ch5, value_ch6, actual_battery_v);
    Serial.println();
  }
}
