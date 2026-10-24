#include "MPU6050_Sensor.h" // เรียกใช้ไฟล์ .h เวอร์ชันใหม่ที่ทำ Filter เอง
#include "BLE_Manager.h"
#include <esp_task_wdt.h> 

#define ENABLE_SERIAL_PLOTTER 1 
#define ENABLE_CALIBRATION false 

#define SDA_PIN 6
#define SCL_PIN 7

MPU_Sensor mpu1;
MPU_Sensor mpu2;
IMU_BLE_Data bleData;

hw_timer_t * timer = NULL;
volatile bool dataReady = false;

// ฟังก์ชัน Interrupt (ISR) 
void IRAM_ATTR onTimer() {
  dataReady = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Serial.println("=========================================");
  Serial.println("System Booting...");
  Serial.println("Please KEEP SENSOR STILL for Calibration.");
  Serial.println("=========================================");

  Serial.println("Init MPU 1 (Addr: 0x68)...");
  MPU_Init(mpu1, Wire, 0x68, SDA_PIN, SCL_PIN, ENABLE_CALIBRATION);
  
  Serial.println("Init MPU 2 (Addr: 0x69)...");
  MPU_Init(mpu2, Wire, 0x69, SDA_PIN, SCL_PIN, ENABLE_CALIBRATION);

  // ==========================================
  // [ใหม่] ปรับค่า Beta อย่างอิสระได้ที่นี่
  // ==========================================
  mpu1.beta = 0.05f; // MPU1: 0.05 กราฟจะสมูท นิ่ง ลื่นไหล (เหมาะกับโมเดล 3D)
  mpu2.beta = 0.05f; // MPU2: ปรับแยกกันได้ตามต้องการ

  Serial.println("Calibration completed. Stabilizing sensor for 15 seconds...");
  for (int i = 15; i > 0; i--) {
    Serial.print(i); Serial.println(" seconds remaining...");
    delay(1000);
  }

  Serial.println("Starting BLE Server...");
  BLE_Init();

  #if ENABLE_SERIAL_PLOTTER
    // ปริ้นท์ Legend ให้ Serial Plotter เพื่อดูกราฟเปรียบเทียบ (ก่อน vs หลังเข้า Filter)
    Serial.println("RawPitch1,FilteredPitch1,RawRoll1,FilteredRoll1");
  #endif

  // ตั้งค่า Task Watchdog Timer
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 3000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, 
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  // ตั้งค่า Hardware Timer (400Hz)
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 2500, true, 0); 
  
  Serial.println("System Ready & BLE Advertising Started!");
}

void loop() {
  esp_task_wdt_reset(); 
  BLE_HandleConnection();

  // ลูปจะทำงานทุกๆ 2.5ms (400Hz) ตามที่ Timer สั่ง
  if (dataReady) {
    dataReady = false; 

    MPU_Update(mpu1);
    MPU_Update(mpu2);

    // แพ็กข้อมูลส่งเข้า BLE (ใช้เฉพาะค่ามุมที่ผ่าน Filter แล้วเพื่อให้โมเดล 3D นิ่ง)
    bleData.roll1  = mpu1.roll;
    bleData.pitch1 = mpu1.pitch;
    bleData.yaw1   = mpu1.yaw;
    
    bleData.roll2  = mpu2.roll;
    bleData.pitch2 = mpu2.pitch;
    bleData.yaw2   = mpu2.yaw;
    
    BLE_SendData(bleData);

    // ส่งค่าเข้า Serial Plotter สำหรับการ Debug ค่า Beta
    #if ENABLE_SERIAL_PLOTTER
      Serial.print(bleData.roll1);  Serial.print(",");
      Serial.print(bleData.pitch1); Serial.print(",");
      Serial.print(bleData.yaw1);   Serial.print(",");
      Serial.print(bleData.roll2);  Serial.print(",");
      Serial.print(bleData.pitch2); Serial.print(",");
      Serial.println(bleData.yaw2);
    #endif
  }
}