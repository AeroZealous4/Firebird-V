#include <Arduino.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define LED 2
#define BUTTON 0
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define LED_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUTTON_CHARACTERISTIC_UUID "8801f158-f55e-4550-95f6-d260381b99e7"

BLECharacteristic *ledCharacteristic;
BLECharacteristic *buttonCharacteristic;

bool deviceConnected = false;
volatile int buttonState = HIGH;

void pin_ISR();


class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Central connected XD");
      deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
      Serial.println("Central dis-connected :(");
      deviceConnected = false;
    }
};


class ControlSwitch: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      Serial.printf("Received '%s'\n", value.c_str());
      if (value.length() > 1) {
        Serial.println("Value should be 0 or 1");
        return;
      }
      long switchState = std::strtol(value.c_str(), NULL, 10);
      if (switchState == 0l) {
        digitalWrite(LED, LOW);
      }
      else {
        digitalWrite(LED, HIGH);
      }
    }
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  // set up pin
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), pin_ISR, CHANGE);

  // set up ble
  BLEDevice::init("esp32-two-way");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *lightSwitchService = pServer->createService(SERVICE_UUID);
  ledCharacteristic = lightSwitchService->createCharacteristic(
                            LED_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE
                          );
  BLEDescriptor* descriptor = new BLEDescriptor(BLEUUID((uint16_t) 0x2901));
  descriptor->setValue("Set LED");
  ledCharacteristic->addDescriptor(descriptor);
  ledCharacteristic->addDescriptor(new BLE2902());
  ledCharacteristic->setCallbacks(new ControlSwitch());

  buttonCharacteristic = lightSwitchService->createCharacteristic(
                          BUTTON_CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  // client charactersitic descriptor: required for notify
  buttonCharacteristic->addDescriptor(new BLE2902());
  lightSwitchService->start();

  // off initially
  ledCharacteristic->setValue("0");
  buttonCharacteristic->setValue("0");

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}


void loop() {
  delay(1000);
  if (buttonState == LOW) {
    Serial.println("Button pressed!");
    if (buttonCharacteristic->getValue() == "0") {
      // Serial.println("Button pressed!");
      buttonCharacteristic->setValue("1");
      buttonCharacteristic->notify();
    }
    else {
      buttonCharacteristic->setValue("0");
      buttonCharacteristic->notify();
    }
  }
}


void pin_ISR() {
  buttonState = digitalRead(BUTTON);
}
