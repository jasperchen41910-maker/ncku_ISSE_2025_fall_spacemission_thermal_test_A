#include <Wire.h>
#include <Adafruit_GPS.h>
#include "SparkFun_ISM330DHCX.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h> 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Arduino.h>
#include "Mcp320x.h"
#include <DFRobot_Geiger.h>

// --- 1. 硬體定義 ---
#define LoRaSerial Serial2
#define GPSSerial  Serial6
#define ONE_WIRE_BUS 2

// --- 計時器變數 ---
unsigned long timerDataSend = 0;
const long intervalDataSend = 2000;   

// 溫度讀取專用計時
unsigned long timerTempRequest = 0;
const long delayTempWait = 800; // DS18B20 轉換需要約 750ms
bool tempRequested = false;
float currentTempC = 0.0;

// --- 2. 物件宣告 ---
Adafruit_GPS GPS(&GPSSerial);
SparkFun_ISM330DHCX myISM; 
SFE_MMC5983MA myMag;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- ADC ---
#define CS1 0
#define CLK 1000000
#define ADC_VREF 5000  
MCP3208 adc1(ADC_VREF, CS1, &SPI1);

// --- 3. IMU 全域變數 ---
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll_qua_deg = 0, pitch_qua_deg = 0, yaw_qua_deg = 0;
float bx = 0, by = 0, bz = 0;

unsigned long last_imu_time_micros = 0;

// --- Geiger counter ---
DFRobot_Geiger geiger(22);
unsigned long global_conts = 0;
unsigned long cpm_start_time = 0;      
unsigned long cpm_20s_accumulator = 0; // 改為 20秒累積一次，配合傳輸
float calculated_cpm = 0.0;            

// --- 緩衝區 (取代 String) ---
char packetBuffer[256]; 

void setup() {
  Serial.begin(115200);
  LoRaSerial.begin(9600);

  // GPS Setup
  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  Wire.begin();
  delay(100);

  // Temp Setup (非阻塞模式)
  sensors.begin();
  sensors.setWaitForConversion(false); // ★ 關鍵：不等待轉換完成
  sensors.requestTemperatures();       // 第一次請求
  tempRequested = true;
  timerTempRequest = millis();

  // IMU Setup
  if (!myISM.begin()) {
    Serial.println("❌ ISM330DHCX not found!");
    while (1);
  }
  myISM.deviceReset();
  while (!myISM.getDeviceReset()) delay(1);
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

  // SPI / ADC Setup
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPISettings settings(CLK, MSBFIRST, SPI_MODE0);
  SPI1.setMISO(1);
  SPI1.begin();
  SPI1.beginTransaction(settings);

  // Mag Setup
  if (!myMag.begin()) {
    Serial.println("❌ MMC5983MA not found!");
    while (true);
  }
  myMag.softReset();

  Serial.println("=== System Started ===");
  last_imu_time_micros = micros();

  geiger.start();
  cpm_start_time = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // ================= 1. GPS Parsing (持續進行) =================
  // 使用庫內建的讀取機制，不要自己手動搬字串，效率較高
  GPS.read(); 
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      // 解析失敗，不做事
    }
  }

  // ================= 2. 溫度讀取 (非阻塞狀態機) =================
  // 如果已經請求過溫度，且時間超過 800ms，則讀取數值
  if (tempRequested && (currentMillis - timerTempRequest >= delayTempWait)) {
    currentTempC = sensors.getTempCByIndex(0);
    tempRequested = false; // 標記為已讀取，等待下一次 request
  }

  // ================= 3. IMU 計算 (高頻率) =================
  // 只有當 IMU 真的有新數據時才計算，但 DeltaT 必須正確
  if (myISM.checkStatus()) {
    unsigned long current_micros = micros();
    // ★ 修正 DeltaT 計算：確保時間差合理，避免溢位或過大
    float DeltaT = (current_micros - last_imu_time_micros) / 1000000.0f;
    last_imu_time_micros = current_micros;

    // 簡單濾波：如果 DeltaT 太大 (例如因為傳輸資料卡住)，限制它以免積分發散
    if (DeltaT > 0.1f) DeltaT = 0.01f; 

    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    // --- 磁力計讀取 ---
    uint32_t currentX = myMag.getMeasurementX();
    uint32_t currentY = myMag.getMeasurementY();
    uint32_t currentZ = myMag.getMeasurementZ();
    
    // 你的校正參數
    double scaledX = -((double)currentX - 133872)*0.7277/131072.0;
    double scaledY = -((double)currentY - 131761)*1.0/131072.0;
    double scaledZ = -((double)currentZ - 132287)*1.1343/131072.0;
    bx = scaledX * 800; by = scaledY * 800; bz = scaledZ * 800;

    // --- 加速度與陀螺儀單位轉換 ---
    float accelX_g = (float)accelData.xData / 989.0;
    float accelY_g = (float)accelData.yData / 1009.48;
    float accelZ_g = (float)accelData.zData / 1008.0;

    float gX_dps = (float)gyroData.xData * 0.000872 - 0.183;
    float gY_dps = (float)gyroData.yData * 0.000872 + 0.3;
    float gZ_dps = 1.09 * ((float)gyroData.zData * 0.000872 - 0.061);

    // --- 姿態解算 (Madgwick 核心邏輯) ---
    float wx = gX_dps * PI / 180.0f;
    float wy = gY_dps * PI / 180.0f;
    float wz = gZ_dps * PI / 180.0f;

    float q0_dot = 0.5f * (-q1*wx - q2*wy - q3*wz);
    float q1_dot = 0.5f * ( q0*wx + q2*wz - q3*wy);
    float q2_dot = 0.5f * ( q0*wy - q1*wz + q3*wx);
    float q3_dot = 0.5f * ( q0*wz + q1*wy - q2*wx);

    q0 += q0_dot * DeltaT;
    q1 += q1_dot * DeltaT;
    q2 += q2_dot * DeltaT;
    q3 += q3_dot * DeltaT;

    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0f) {
      q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }

    // 計算歐拉角 (Quaternion to Euler)
    float pre_pitch = 2 * (q0*q2 - q1*q3);
    pre_pitch = constrain(pre_pitch, -1.0f, 1.0f); // 避免 asin 超出範圍
    
    float pre_yaw_1 = (q0*q3 + q1*q2);
    float pre_yaw_2 = (1 - 2*(q2*q2 + q3*q3));
    float pre_roll_1 = 2 * (q0*q1 + q2*q3);
    float pre_roll_2 = (1 - 2*(q1*q1 + q2*q2));

    pitch_qua_deg = asin(pre_pitch) * (180.0f/PI);
    roll_qua_deg = atan2(pre_roll_1, pre_roll_2) * (180.0f/PI);
    yaw_qua_deg = atan2(pre_yaw_1, pre_yaw_2) * (180.0f/PI);

    // 簡單的互補濾波修正 (利用重力加速度修正 Roll/Pitch)
    float total_g = sqrt(accelX_g*accelX_g + accelY_g*accelY_g + accelZ_g*accelZ_g);
    if (total_g > 0.9 && total_g < 1.1) { // 只有在靜止或平穩時才修正
        float roll_acc = atan2(accelY_g, accelZ_g) * (180.0f/PI);
        float pitch_acc = asin(constrain(-accelX_g / total_g, -1.0, 1.0)) * (180.0f/PI);
        
        // 簡單融合，權重可調整
        if (abs(roll_acc - roll_qua_deg) > 5.0) roll_qua_deg = roll_qua_deg * 0.98 + roll_acc * 0.02;
        if (abs(pitch_acc - pitch_qua_deg) > 5.0) pitch_qua_deg = pitch_qua_deg * 0.98 + pitch_acc * 0.02;
    }
  }

     // ================= 4. 定時傳輸 (每 2 秒) =================
  if (currentMillis - timerDataSend >= intervalDataSend) {
    timerDataSend = currentMillis;

    // 1. 暫停蓋革 (抗干擾)
    geiger.pause(); 

    // 2. 準備資料
    if (!tempRequested) {
        sensors.requestTemperatures();
        timerTempRequest = millis();
        tempRequested = true;
    }

    // --- ADC 完整讀取 (配合地面站需求) ---
    // 讀取所有地面站需要的通道
    float raw_ch0 = adc1.read(MCP3208::Channel::SINGLE_0); // Batt
    float raw_ch3 = adc1.read(MCP3208::Channel::SINGLE_3); // 5V
    float raw_ch4 = adc1.read(MCP3208::Channel::SINGLE_4); // 3.3V eFuse
    float raw_ch5 = adc1.read(MCP3208::Channel::SINGLE_5); // 3.3V
    float raw_ch6 = adc1.read(MCP3208::Channel::SINGLE_6); // GND

    // 轉換電壓 (假設都是直接讀取或有分壓，這裡還原你原本的邏輯)
    // 注意：這裡依照你原本程式碼邏輯，除以 1000 轉 V
    float val_batt = (adc1.toAnalog(raw_ch0)/1000.0) * (4.7f + 6.8f) / 6.8f; 
    float val_50   = adc1.toAnalog(raw_ch3)/1000.0;
    float val_efuse= adc1.toAnalog(raw_ch4)/1000.0;
    float val_33   = adc1.toAnalog(raw_ch5)/1000.0;
    float val_gnd  = adc1.toAnalog(raw_ch6)/1000.0;

    // Geiger 計算
    unsigned long conts = geiger.getTotalCount(); 
    global_conts += conts;
    cpm_20s_accumulator += conts;
    if (millis() - cpm_start_time >= 20000) {
        calculated_cpm = (float)cpm_20s_accumulator * 3.0f; 
        cpm_20s_accumulator = 0;
        cpm_start_time = millis();
    }

    // --- 3. 組合封包 (修正格式 mismatch) ---
    
    // (A) GPS Packet - 修正：插入 1.0 作為假的 HDOP，讓 Altitude 落在 parts[9]
    if (GPS.fix) {
        snprintf(packetBuffer, sizeof(packetBuffer), "$GPGGA,%02d%02d%02d,%f,%c,%f,%c,%d,%d,1.0,%.2f,M\n", 
                 GPS.hour, GPS.minute, GPS.seconds, GPS.latitude, GPS.lat, GPS.longitude, GPS.lon, GPS.fix, GPS.satellites, GPS.altitude);
    } else {
        snprintf(packetBuffer, sizeof(packetBuffer), "$GPGGA,NoFix,Sat:%d\n", (int)GPS.satellites);
    }
    LoRaSerial.print(packetBuffer);
    Serial.print(packetBuffer);

    // (B) IMU Packet - 格式正確
    snprintf(packetBuffer, sizeof(packetBuffer), "$IMU,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
             pitch_qua_deg, roll_qua_deg, yaw_qua_deg, q0, q1, q2, q3, bx, by, bz);
    LoRaSerial.print(packetBuffer);
    Serial.print(packetBuffer);

    // (C) Temp Packet - 格式正確
    snprintf(packetBuffer, sizeof(packetBuffer), "$Temp,%.2f\n", currentTempC);
    LoRaSerial.print(packetBuffer);
    Serial.print(packetBuffer);

    // (D) ADC Packet - 修正：補齊 5 個數值
    // 順序依照 Python: Batt, 3.3V, 5V, eFuse, GND
    snprintf(packetBuffer, sizeof(packetBuffer), "$ADC,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
             val_batt, val_33, val_50, val_efuse, val_gnd);
    LoRaSerial.print(packetBuffer);
    Serial.print(packetBuffer);

    // (E) Geiger Packet - 格式正確
    snprintf(packetBuffer, sizeof(packetBuffer), "$Geg,%.2f,%.2f,%lu,%02d%02d%02d\n",
             calculated_cpm, GPS.altitude, global_conts, GPS.hour, GPS.minute, GPS.seconds);
    LoRaSerial.print(packetBuffer);
    Serial.print(packetBuffer);

    // 4. 等待傳輸完成並恢復
    LoRaSerial.flush(); 
    geiger.start();
    last_imu_time_micros = micros(); 
  }

}