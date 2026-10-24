#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BLE_SERVER_NAME "ESP32_IMU_DUAL_2"
#define SERVICE_UUID        "1d67f95e-6a29-4634-9a9b-4d15298d83fe"
#define CHARACTERISTIC_UUID "4d7695cf-ec74-426c-ad51-a726b0842b6b"

// โครงสร้างข้อมูลส่งผ่าน BLE (บีบอัดหน่วยความจำ)
struct __attribute__((packed)) IMU_BLE_Data {
    float roll1, pitch1, yaw1; 
    float roll2, pitch2, yaw2; 
};

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void BLE_Init() {
  BLEDevice::init(BLE_SERVER_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
}

void BLE_HandleConnection() {
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising(); 
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}

void BLE_SendData(IMU_BLE_Data &data) {
    if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&data, sizeof(IMU_BLE_Data));
        pCharacteristic->notify();
    }
}

#endif