#include "MPU6050_Sensor.h" 

#define I2C_SDA 6
#define I2C_SCL 7
#define ENABLE_CALIBRATION false

MPU_Sensor mpu1;
MPU_Sensor mpu2;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Init MPU 1 (Addr: 0x68)...");
  MPU_Init(mpu1, Wire, 0x68, I2C_SDA, I2C_SCL, ENABLE_CALIBRATION);
  
  Serial.println("Init MPU 2 (Addr: 0x69)...");
  MPU_Init(mpu2, Wire, 0x69, I2C_SDA, I2C_SCL, ENABLE_CALIBRATION);

  Serial.println("System Ready!");
}

// void loop() {
//   MPU_Update(mpu1);
//   MPU_Update(mpu2);

//   // ==========================================
//   // เลือกกลุ่มข้อมูลที่ต้องการแสดงผลใน Serial Plotter
//   // (แนะนำให้เปิดทีละกลุ่ม เพื่อไม่ให้สเกลกราฟเพี้ยน)
//   // ==========================================

//   // --- ตัวเลือกที่ 1: ดูมุม Pitch แบบดิบ (หาจาก Accel ตรงๆ) เทียบกับ Pitch ที่ผ่าน Madgwick ---

//   // float raw_pitch = atan2(-mpu1.raw_ax, sqrt(pow(mpu1.raw_ay, 2) + pow(mpu1.raw_az, 2))) * 180.0 / PI;
//   // Serial.print("RawPitch:");   Serial.print(raw_pitch);   Serial.print(",");
//   // Serial.print("MadgPitch:");  Serial.print(mpu1.pitch); 

//   // --- ตัวเลือกที่ 2: ดูค่าความเร่งดิบแกน X เทียบกับมุม Pitch ---
//   // (เพื่อดูว่าพอมีความเร่งแล้วมุมเพี้ยนไหม)

//   // Serial.print("RawAccelX(g):"); Serial.print(mpu1.raw_ax * 90.0); Serial.print(","); // คูณ 90 เพื่อดันสเกลให้ใกล้เคียงมุม
//   // Serial.print("MadgPitch:");    Serial.print(mpu1.pitch);

//   // --- ตัวเลือกที่ 3: ดูค่า Output ทั้ง 6 ตัวของ Madgwick ---
  
//   // Serial.print("Pitch:"); Serial.print(mpu1.pitch); Serial.print(",");
//   // Serial.print("Roll:");  Serial.print(mpu1.roll);  Serial.print(",");
//   // Serial.print("Yaw:");   Serial.print(mpu1.yaw);   Serial.print(",");
  
//   Serial.print("PosX:");  Serial.print(mpu1.posX * 100.0); Serial.print(",");
//   Serial.print("PosY:");  Serial.print(mpu1.posY * 100.0); Serial.print(",");
//   Serial.print("PosZ:");  Serial.print(mpu1.posZ * 100.0); Serial.print(",");

//   Serial.print("VelX:");  Serial.print(mpu1.velX); Serial.print(",");
//   Serial.print("VelY:");  Serial.print(mpu1.velY); Serial.print(",");
//   Serial.print("VelZ:");  Serial.print(mpu1.velZ); Serial.print(",");

// Serial.print("LinAccX:"); Serial.print(mpu1.lin_ax); Serial.print(",");
// Serial.print("LinAccY:"); Serial.print(mpu1.lin_ay); Serial.print(",");
// Serial.print("LinAccZ:"); Serial.print(mpu1.lin_az); Serial.print(",");

//   Serial.println();
//   // delay(5); 
// }
void loop() {
  MPU_Update(mpu1);
  // ส่งข้อมูลให้ Python (ตัวเลขล้วน คั่นด้วยลูกน้ำ)
  Serial.print(mpu1.pitch); Serial.print(",");
  Serial.print(mpu1.roll);  Serial.print(",");
  Serial.print(mpu1.yaw);   Serial.print(",");
  Serial.print(mpu1.posX);  Serial.print(",");
  Serial.print(mpu1.posY);  Serial.print(",");
  Serial.println(mpu1.posZ);

  delay(10); // อัปเดตที่ 100Hz
}