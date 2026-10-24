#include <Wire.h>
#include <math.h>

// ==========================================
// [TUNING SECTION] รวมจุดปรับจูนค่าพารามิเตอร์
// ==========================================
#define TUNE_MADGWICK_BETA 0.05f 
#define TUNE_ACCEL_DEADZONE 0.05
#define TUNE_VELOCITY_DAMPING 0.90
#define TUNE_CALIB_SAMPLES 500

struct MPU_Sensor {
  TwoWire *wire;
  uint8_t addr; 
  
  // [ใหม่] ตัวแปรสำหรับ Madgwick Filter แบบทำเอง
  float beta = TUNE_MADGWICK_BETA;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  
  float bias_AcX = 0, bias_AcY = 0, bias_AcZ = 0;
  float bias_GyX = 0, bias_GyY = 0, bias_GyZ = 0;
  
  float raw_ax = 0, raw_ay = 0, raw_az = 0;
  float raw_gx = 0, raw_gy = 0, raw_gz = 0;
  float lin_ax = 0, lin_ay = 0, lin_az = 0;

  // [OUTPUT]
  float raw_pitch = 0, raw_roll = 0; // มุมดิบก่อนเข้า Filter
  float pitch = 0, roll = 0, yaw = 0; // มุมหลังผ่าน Filter
  
  float posX = 0, posY = 0, posZ = 0;
  float velX = 0, velY = 0, velZ = 0;
  
  unsigned long prevTime = 0;
};

void MPU_Init(MPU_Sensor &sensor, TwoWire &w, uint8_t address, int sda, int scl, bool calibrate) {
  sensor.wire = &w;
  sensor.addr = address;
  
  w.begin(sda, scl);
  w.beginTransmission(sensor.addr);
  w.write(0x6B);
  w.write(0x00);
  w.endTransmission(true);
  delay(100);

  w.beginTransmission(sensor.addr);
  w.write(0x1C); w.write(0x00); 
  w.endTransmission(true);
  w.beginTransmission(sensor.addr);
  w.write(0x1B); w.write(0x00); 
  w.endTransmission(true);

  if (calibrate) {
    long sumAcX = 0, sumAcY = 0, sumAcZ = 0;
    long sumGyX = 0, sumGyY = 0, sumGyZ = 0;
    
    for (int i = 0; i < TUNE_CALIB_SAMPLES; i++) {
      w.beginTransmission(sensor.addr);
      w.write(0x3B);
      w.endTransmission(false);
      w.requestFrom((uint16_t)sensor.addr, (uint8_t)14, true);
      
      sumAcX += w.read()<<8 | w.read();  
      sumAcY += w.read()<<8 | w.read();  
      sumAcZ += w.read()<<8 | w.read();  
      w.read()<<8 | w.read(); 
      sumGyX += w.read()<<8 | w.read();  
      sumGyY += w.read()<<8 | w.read();  
      sumGyZ += w.read()<<8 | w.read();  
      delay(2);
    }
    sensor.bias_AcX = (float)sumAcX / TUNE_CALIB_SAMPLES;
    sensor.bias_AcY = (float)sumAcY / TUNE_CALIB_SAMPLES;
    sensor.bias_AcZ = ((float)sumAcZ / TUNE_CALIB_SAMPLES); 
    
    sensor.bias_GyX = (float)sumGyX / TUNE_CALIB_SAMPLES;
    sensor.bias_GyY = (float)sumGyY / TUNE_CALIB_SAMPLES;
    sensor.bias_GyZ = (float)sumGyZ / TUNE_CALIB_SAMPLES;
  } 
  
  sensor.q0 = 1.0f; sensor.q1 = 0.0f; sensor.q2 = 0.0f; sensor.q3 = 0.0f;
  sensor.prevTime = micros();
}

void MPU_Update(MPU_Sensor &sensor) {
  sensor.wire->beginTransmission(sensor.addr);
  sensor.wire->write(0x3B);
  sensor.wire->endTransmission(false);
  sensor.wire->requestFrom((uint16_t)sensor.addr, (uint8_t)14, true);
  
  if(sensor.wire->available() == 14) {
    int16_t rawAcX = sensor.wire->read()<<8 | sensor.wire->read();  
    int16_t rawAcY = sensor.wire->read()<<8 | sensor.wire->read();  
    int16_t rawAcZ = sensor.wire->read()<<8 | sensor.wire->read();  
    sensor.wire->read()<<8 | sensor.wire->read();  
    int16_t rawGyX = sensor.wire->read()<<8 | sensor.wire->read();  
    int16_t rawGyY = sensor.wire->read()<<8 | sensor.wire->read();  
    int16_t rawGyZ = sensor.wire->read()<<8 | sensor.wire->read();  

    unsigned long currentTime = micros();
    float dt = (currentTime - sensor.prevTime) / 1000000.0f;
    sensor.prevTime = currentTime;
    
    if(dt <= 0.0f) return;

    sensor.raw_ax = (rawAcX - sensor.bias_AcX) / 16384.0f;
    sensor.raw_ay = (rawAcY - sensor.bias_AcY) / 16384.0f;
    sensor.raw_az = (rawAcZ - sensor.bias_AcZ) / 16384.0f;

    sensor.raw_gx = (rawGyX - sensor.bias_GyX) / 131.0f;
    sensor.raw_gy = (rawGyY - sensor.bias_GyY) / 131.0f;
    sensor.raw_gz = (rawGyZ - sensor.bias_GyZ) / 131.0f;

    // 1. คำนวณมุมก่อนเข้า Filter (Raw Pitch & Roll จากความเร่ง)
    sensor.raw_pitch = atan2(-sensor.raw_ax, sqrt(sensor.raw_ay * sensor.raw_ay + sensor.raw_az * sensor.raw_az)) * 57.29577951f;
    sensor.raw_roll  = atan2(sensor.raw_ay, sensor.raw_az) * 57.29577951f;

    // 2. Custom Madgwick Filter
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // แปลงหน่วย Gyro เป็น rad/s
    float gx = sensor.raw_gx * 0.0174533f;
    float gy = sensor.raw_gy * 0.0174533f;
    float gz = sensor.raw_gz * 0.0174533f;

    qDot1 = 0.5f * (-sensor.q1 * gx - sensor.q2 * gy - sensor.q3 * gz);
    qDot2 = 0.5f * (sensor.q0 * gx + sensor.q2 * gz - sensor.q3 * gy);
    qDot3 = 0.5f * (sensor.q0 * gy - sensor.q1 * gz + sensor.q3 * gx);
    qDot4 = 0.5f * (sensor.q0 * gz + sensor.q1 * gy - sensor.q2 * gx);

    if(!((sensor.raw_ax == 0.0f) && (sensor.raw_ay == 0.0f) && (sensor.raw_az == 0.0f))) {
      recipNorm = 1.0f / sqrt(sensor.raw_ax * sensor.raw_ax + sensor.raw_ay * sensor.raw_ay + sensor.raw_az * sensor.raw_az);
      float ax = sensor.raw_ax * recipNorm;
      float ay = sensor.raw_ay * recipNorm;
      float az = sensor.raw_az * recipNorm;

      _2q0 = 2.0f * sensor.q0; _2q1 = 2.0f * sensor.q1; _2q2 = 2.0f * sensor.q2; _2q3 = 2.0f * sensor.q3;
      _4q0 = 4.0f * sensor.q0; _4q1 = 4.0f * sensor.q1; _4q2 = 4.0f * sensor.q2;
      _8q1 = 8.0f * sensor.q1; _8q2 = 8.0f * sensor.q2;
      q0q0 = sensor.q0 * sensor.q0; q1q1 = sensor.q1 * sensor.q1;
      q2q2 = sensor.q2 * sensor.q2; q3q3 = sensor.q3 * sensor.q3;

      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * sensor.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * sensor.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * sensor.q3 - _2q1 * ax + 4.0f * q2q2 * sensor.q3 - _2q2 * ay;

      recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

      qDot1 -= sensor.beta * s0;
      qDot2 -= sensor.beta * s1;
      qDot3 -= sensor.beta * s2;
      qDot4 -= sensor.beta * s3;
    }

    sensor.q0 += qDot1 * dt; sensor.q1 += qDot2 * dt; sensor.q2 += qDot3 * dt; sensor.q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrt(sensor.q0 * sensor.q0 + sensor.q1 * sensor.q1 + sensor.q2 * sensor.q2 + sensor.q3 * sensor.q3);
    sensor.q0 *= recipNorm; sensor.q1 *= recipNorm; sensor.q2 *= recipNorm; sensor.q3 *= recipNorm;

    // 3. แปลงกลับเป็นมุม Euler
    sensor.roll  = atan2(2.0f * (sensor.q0 * sensor.q1 + sensor.q2 * sensor.q3), 1.0f - 2.0f * (sensor.q1 * sensor.q1 + sensor.q2 * sensor.q2)) * 57.29577951f;
    sensor.pitch = asin(-2.0f * (sensor.q1 * sensor.q3 - sensor.q0 * sensor.q2)) * 57.29577951f;
    sensor.yaw   = atan2(2.0f * (sensor.q1 * sensor.q2 + sensor.q0 * sensor.q3), 1.0f - 2.0f * (sensor.q2 * sensor.q2 + sensor.q3 * sensor.q3)) * 57.29577951f;

    // 4. หักลบแรงโน้มถ่วง (คงเดิมจากไฟล์ที่ส่งมา)
    float p_rad = sensor.pitch * PI / 180.0f;
    float r_rad = sensor.roll * PI / 180.0f;
    float grav_x = -sin(p_rad);
    float grav_y = cos(p_rad) * sin(r_rad);
    float grav_z = cos(p_rad) * cos(r_rad);

    sensor.lin_ax = (sensor.raw_ax - grav_x) * 9.81f;
    sensor.lin_ay = (sensor.raw_ay - grav_y) * 9.81f;
    sensor.lin_az = (sensor.raw_az - grav_z) * 9.81f;

    // 5. ZUPT & Deadzone (คงเดิมจากไฟล์ที่ส่งมา)
    float ax_calc = sensor.lin_ax; float ay_calc = sensor.lin_ay; float az_calc = sensor.lin_az;
    float acc_mag = sqrt(ax_calc*ax_calc + ay_calc*ay_calc + az_calc*az_calc);

    if (acc_mag < TUNE_ACCEL_DEADZONE * 9.81f) {
      ax_calc = 0; ay_calc = 0; az_calc = 0;
      sensor.velX = 0; sensor.velY = 0; sensor.velZ = 0;
    } else {
      sensor.velX *= 0.90f; sensor.velY *= 0.90f; sensor.velZ *= 0.90f;
    }

    sensor.velX += ax_calc * dt; sensor.velY += ay_calc * dt; sensor.velZ += az_calc * dt;
    float vel_mag = sqrt(sensor.velX*sensor.velX + sensor.velY*sensor.velY + sensor.velZ*sensor.velZ);

    if (vel_mag < TUNE_ACCEL_DEADZONE * 9.81f) {
      sensor.velX = 0; sensor.velY = 0; sensor.velZ = 0;
    } else {
      sensor.posX *= 0.90f; sensor.posY *= 0.90f; sensor.posZ *= 0.90f;
    }

    sensor.posX += sensor.velX * dt; sensor.posY += sensor.velY * dt; sensor.posZ += sensor.velZ * dt;
  }
}

void MPU_ResetPosition(MPU_Sensor &sensor) {
  sensor.posX = 0; sensor.posY = 0; sensor.posZ = 0;
  sensor.velX = 0; sensor.velY = 0; sensor.velZ = 0;
}