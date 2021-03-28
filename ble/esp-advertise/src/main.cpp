#include "sys/time.h"

#include "Arduino.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"

static uint32_t count; // remember number of boots in RTC Memory

BLEAdvertising *pAdvertising;

#define TEMP_UUID "d1ff7171-6437-4a37-a9f5-1587c8a6f9d7"

void setAdvertisement() {
  char buffer[32] = "";
  sprintf(buffer, "00%d", count & 0xFFFF);
  std::string data = std::string(buffer, strlen(buffer));
  
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setAppearance(ESP_BLE_APPEARANCE_GENERIC_THERMOMETER);
  oAdvertisementData.setManufacturerData(data);
  // pAdvertising->addServiceUUID(BEACON_UUID);
  // oAdvertisementData.setServiceData(BLEUUID(COUNT_UUID), "33.05");
  oAdvertisementData.setFlags(ESP_BLE_ADV_FLAG_LIMIT_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  // pAdvertising->m_advParams.adv_type
  Serial.printf("Len of adv. data: %d\n", oAdvertisementData.getPayload().length());
  // pAdvertising->setScanResponseData(oScanResponseData);
}

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");
  pAdvertising = BLEDevice::getAdvertising();
  
  setAdvertisement();
   // Start advertising
  pAdvertising->start();
  Serial.println("Advertizing started...");
  delay(100);
  // pAdvertising->stop();
}

void loop() {
  setAdvertisement();
  delay(1000);
  count++;
}
