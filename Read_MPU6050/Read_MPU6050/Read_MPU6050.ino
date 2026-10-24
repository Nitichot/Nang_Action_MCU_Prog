#include "MPU6050_Sensor.h" 

#define I2C_SDA 6
#define I2C_SCL 7
#define ENABLE_CALIBRATION false // แนะนำให้เปิดเพื่อให้กราฟนิ่ง

MPU_Sensor mpu1;
MPU_Sensor mpu2;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Init MPU 1 (Addr: 0x68)...");
  MPU_Init(mpu1, Wire, 0x68, I2C_SDA, I2C_SCL, ENABLE_CALIBRATION);
  
  Serial.println("Init MPU 2 (Addr: 0x69)...");
  MPU_Init(mpu2, Wire, 0x69, I2C_SDA, I2C_SCL, ENABLE_CALIBRATION);

  // ==========================================
  // [ใหม่] ปรับค่า Beta ตรงนี้ได้เลย!
  // ==========================================
  mpu1.beta = 0.05f; // ค่าต่ำ = กราฟสมูท (เหมาะกับโมเดล 3D)
  mpu2.beta = 0.50f; // ค่าสูง = กราฟแกว่งไว (เข้าหาแรงโน้มถ่วงเร็ว)

  Serial.println("System Ready!");
  
  // ปริ้นท์ชื่อหัวข้อกราฟเพื่อให้ Serial Plotter มี Legend สีบอกว่าเส้นไหนคืออะไร
  Serial.println("RawPitch,FilteredPitch,RawRoll,FilteredRoll");
}

void loop() {
  MPU_Update(mpu1);
  MPU_Update(mpu2);
  
  // ==========================================
  // ส่งข้อมูลเข้า Serial Plotter (เปรียบเทียบ ก่อน/หลัง เข้า Filter)
  // ==========================================
  Serial.print(mpu1.raw_pitch); Serial.print(",");
  Serial.print(mpu1.pitch);     Serial.print(",");
  Serial.print(mpu1.raw_roll);  Serial.print(",");
  Serial.println(mpu1.roll);

  delay(10); // อัปเดตที่ 100Hz
}