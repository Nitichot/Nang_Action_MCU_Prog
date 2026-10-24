#include <Wire.h>
#include <math.h>

// =========================================================================
// [TUNING SECTION] รวมจุดปรับจูนค่าพารามิเตอร์ทั้งหมด
// =========================================================================

// --- 1. Kalman Filter Tuning สำหรับ "มุม (Orientation)" ---
#define TUNE_Q_ANGLE 0.001f  // ค่าน้อย = เชื่อ Gyro มาก (กราฟสมูท)
#define TUNE_Q_BIAS  0.003f  // ความเร็วในการเรียนรู้ Gyro Bias
#define TUNE_R_MEASURE 0.03f // ค่ามาก = ลดอาการมุมสั่นเวลาโดนกระแทก (ลดความเชื่อใจ Accel)

// --- 2. Kalman Filter Tuning สำหรับ "ตำแหน่ง (Position & Velocity)" ---
#define TUNE_Q_POS_NOISE 0.0001f // Process noise ของตำแหน่ง
#define TUNE_Q_VEL_NOISE 0.001f  // Process noise ของความเร็ว

// Measurement Noise (R) ของความเร็ว
#define TUNE_R_ZUPT_STATIONARY 0.0001f // ตอนอยู่นิ่ง: เชื่อมั่น 99.99% ว่าความเร็วเป็น 0
#define TUNE_R_DRAG_MOVING     100.0f  // ตอนขยับ: เชื่อมั่นน้อยมาก (ทำหน้าที่เป็น Damping หนืดๆ)

// --- 3. Threshold ควบคุมสถานะ ---
#define TUNE_ACCEL_DEADZONE 0.035 // เกณฑ์ตัดสินว่า "อยู่นิ่ง" หรือ "ขยับ" (หน่วย g)
#define TUNE_CALIB_SAMPLES 500    // จำนวนรอบ Calibration ตอนเริ่มทำงาน

// =========================================================================
// โครงสร้าง Discrete Kalman Filter สำหรับ "มุม" (State: Angle, Bias)
// =========================================================================
struct KalmanAngle {
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

    float getAngle(float newAngle, float newRate, float dt) {
        float rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + TUNE_Q_ANGLE);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += TUNE_Q_BIAS * dt;

        float S = P[0][0] + TUNE_R_MEASURE;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle; 
        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

// =========================================================================
// [ใหม่] โครงสร้าง Discrete Kalman Filter สำหรับ "ตำแหน่ง" (State: Pos, Vel)
// =========================================================================
struct KalmanPos {
    float p = 0.0f; // Position
    float v = 0.0f; // Velocity
    float P_cov[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

    void predictAndUpdate(float a, float dt, bool isStationary) {
        // 1. Prediction Step (สมการการเคลื่อนที่)
        p += v * dt + 0.5f * a * dt * dt;
        v += a * dt;

        // อัปเดต Process Covariance Matrix (Q)
        P_cov[0][0] += dt * dt * P_cov[1][1] + TUNE_Q_POS_NOISE;
        P_cov[0][1] += dt * P_cov[1][1];
        P_cov[1][0] += dt * P_cov[1][1];
        P_cov[1][1] += TUNE_Q_VEL_NOISE;

        // 2. Update Step (ใช้ ZUPT เป็น Measurement: ให้สมมติว่าความเร็ววัดได้ = 0 เสมอ)
        float R = isStationary ? TUNE_R_ZUPT_STATIONARY : TUNE_R_DRAG_MOVING;
        
        // H = [0, 1] ทำให้ Innovation (y) คิดเฉพาะจาก Velocity
        float S = P_cov[1][1] + R;
        float K0 = P_cov[0][1] / S; // Gain สำหรับแก้ Position
        float K1 = P_cov[1][1] / S; // Gain สำหรับแก้ Velocity

        float z = 0.0f; // ค่าความเร็วที่ "วัดได้" (บังคับให้เป็น 0)
        float y = z - v; // Error ความเร็ว
        
        // ปรับแก้ State
        p += K0 * y;
        v += K1 * y;

        // อัปเดต Error Covariance (ลดความไม่แน่นอน)
        float P00_temp = P_cov[0][0];
        float P01_temp = P_cov[0][1];
        float P10_temp = P_cov[1][0];
        float P11_temp = P_cov[1][1];
        
        P_cov[0][0] = P00_temp - K0 * P10_temp;
        P_cov[0][1] = P01_temp - K0 * P11_temp;
        P_cov[1][0] = P10_temp - K1 * P10_temp;
        P_cov[1][1] = P11_temp - K1 * P11_temp;
    }
};

// =========================================================================
// โครงสร้างตัวแปรของเซ็นเซอร์
// =========================================================================
struct MPU_Sensor {
  TwoWire *wire;
  uint8_t addr; 
  
  KalmanAngle kalmanPitch;
  KalmanAngle kalmanRoll;
  
  KalmanPos kalmanX;
  KalmanPos kalmanY;
  KalmanPos kalmanZ;
  
  float bias_AcX = 0, bias_AcY = 0, bias_AcZ = 0;
  float bias_GyX = 0, bias_GyY = 0, bias_GyZ = 0;
  
  float raw_ax = 0, raw_ay = 0, raw_az = 0;
  float raw_gx = 0, raw_gy = 0, raw_gz = 0;
  float lin_ax = 0, lin_ay = 0, lin_az = 0;

  float pitch = 0, roll = 0, yaw = 0;
  float posX = 0, posY = 0, posZ = 0;
  
  unsigned long prevTime = 0;
};

// =========================================================================
// ฟังก์ชันทำงานหลัก
// =========================================================================
void MPU_Init(MPU_Sensor &sensor, TwoWire &w, uint8_t address, int sda, int scl, bool calibrate) {
  sensor.wire = &w;
  sensor.addr = address;
  
  w.begin(sda, scl);
  w.beginTransmission(sensor.addr);
  w.write(0x6B); w.write(0x00);
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
    sensor.bias_AcZ = ((float)sumAcZ / TUNE_CALIB_SAMPLES) - 16384.0; 
    
    sensor.bias_GyX = (float)sumGyX / TUNE_CALIB_SAMPLES;
    sensor.bias_GyY = (float)sumGyY / TUNE_CALIB_SAMPLES;
    sensor.bias_GyZ = (float)sumGyZ / TUNE_CALIB_SAMPLES;
  } 
  
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
    float dt = (currentTime - sensor.prevTime) / 1000000.0;
    sensor.prevTime = currentTime;
    
    if (dt <= 0) return;

    sensor.raw_ax = (rawAcX - sensor.bias_AcX) / 16384.0;
    sensor.raw_ay = (rawAcY - sensor.bias_AcY) / 16384.0;
    sensor.raw_az = (rawAcZ - sensor.bias_AcZ) / 16384.0;

    sensor.raw_gx = (rawGyX - sensor.bias_GyX) / 131.0;
    sensor.raw_gy = (rawGyY - sensor.bias_GyY) / 131.0;
    sensor.raw_gz = (rawGyZ - sensor.bias_GyZ) / 131.0;

    // 1. อัปเดตมุมด้วย KalmanAngle
    float accel_pitch = atan2(-sensor.raw_ax, sqrt(sensor.raw_ay * sensor.raw_ay + sensor.raw_az * sensor.raw_az)) * 180.0 / PI;
    float accel_roll = atan2(sensor.raw_ay, sensor.raw_az) * 180.0 / PI;

    sensor.pitch = sensor.kalmanPitch.getAngle(accel_pitch, sensor.raw_gy, dt);
    sensor.roll  = sensor.kalmanRoll.getAngle(accel_roll, sensor.raw_gx, dt);
    sensor.yaw  += sensor.raw_gz * dt; 

    // 2. หักลบแรงโน้มถ่วง
    float p_rad = sensor.pitch * PI / 180.0;
    float r_rad = sensor.roll * PI / 180.0;
    float grav_x = -sin(p_rad);
    float grav_y = cos(p_rad) * sin(r_rad);
    float grav_z = cos(p_rad) * cos(r_rad);

    sensor.lin_ax = (sensor.raw_ax - grav_x) * 9.81;
    sensor.lin_ay = (sensor.raw_ay - grav_y) * 9.81;
    sensor.lin_az = (sensor.raw_az - grav_z) * 9.81;

    // 3. เช็คสถานะ ZUPT (อยู่นิ่งหรือไม่)
    float acc_mag = sqrt(sensor.lin_ax*sensor.lin_ax + sensor.lin_ay*sensor.lin_ay + sensor.lin_az*sensor.lin_az);
    bool isStationary = (acc_mag < TUNE_ACCEL_DEADZONE * 9.81);

    // 4. โยนเข้า Kalman Filter ของตำแหน่งแต่ละแกน
    sensor.kalmanX.predictAndUpdate(sensor.lin_ax, dt, isStationary);
    sensor.kalmanY.predictAndUpdate(sensor.lin_ay, dt, isStationary);
    sensor.kalmanZ.predictAndUpdate(sensor.lin_az, dt, isStationary);

    // ดึงค่าผลลัพธ์มาใช้งาน
    sensor.posX = sensor.kalmanX.p;
    sensor.posY = sensor.kalmanY.p;
    sensor.posZ = sensor.kalmanZ.p;
  }
}

void MPU_ResetPosition(MPU_Sensor &sensor) {
  sensor.kalmanX.p = 0; sensor.kalmanX.v = 0;
  sensor.kalmanY.p = 0; sensor.kalmanY.v = 0;
  sensor.kalmanZ.p = 0; sensor.kalmanZ.v = 0;
  sensor.posX = 0; sensor.posY = 0; sensor.posZ = 0;
}