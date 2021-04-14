/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/



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
#define NOTIFY_TRACK_UUID "beb5483f-36e1-4688-b7f5-ea07361b26a8"
#define NOTIFY_DEBUG_UUID "beb5483a-36e1-4688-b7f5-ea07361b26a8"

#define RXD2 16
#define TXD2 17
char str_msg[] = "ESP string: Info will be appended on this string";
unsigned long int Sec = 0 ;
unsigned long int CompleteIn = 0 ; //Task to be completed in this many Secs
unsigned char led_toggle = 0;

BLECharacteristic *ledCharacteristic;
BLECharacteristic *buttonCharacteristic;
BLECharacteristic *TrackCharacteristic;
BLECharacteristic *DebugCharacteristic;

bool deviceConnected = false;
volatile int buttonState = HIGH;

void pin_ISR();

/**
 * Update connection parameters can be called only after connection has been established
 */
// void BLEServer::updateConnParams(esp_bd_addr_t remote_bda, uint16_t minInterval, uint16_t maxInterval, uint16_t latency, uint16_t timeout) {
// 	esp_ble_conn_update_params_t conn_params;
// 	memcpy(conn_params.bda, remote_bda, sizeof(esp_bd_addr_t));
// 	conn_params.latency = latency;
// 	conn_params.max_int = maxInterval;    // max_int = 0x20*1.25ms = 40ms
// 	conn_params.min_int = minInterval;    // min_int = 0x10*1.25ms = 20ms
// 	conn_params.timeout = timeout;    // timeout = 400*10ms = 4000ms
// 	esp_ble_gap_update_conn_params(&conn_params); 
// }

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      Serial.println("Central connected XD");
      pServer->updateConnParams(param->connect.remote_bda, 0x01, 0x90, 0, 6000);
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
      strcpy(str_msg,value.c_str());
      char *token = strtok(str_msg,"-");
      
      char *ptr;
      // char *cmd;
      char cmd_1[] = "scan";
      char cmd_2[] = "fetch";
      char cmd_1_1[] = "major";
      char cmd_1_2[] = "minor";

      // cmd = token;
      int res1 =  strcmp(token,cmd_1);
      int res2 =  strcmp(token,cmd_2);

      Serial.printf("'%s'\n",token);
      if(res1 == 0) 
      {
        // Serial.printf("scan ");
        if(token!=NULL)
        {
          token = strtok(NULL,"-");
          
          long switchState1 = std::strtol(token, &ptr, 10);
          Serial.printf("Scan plot: %ld\n",switchState1); 
          // Serial.printf("'%s'\n",token);     

          if(switchState1 < 1 || switchState1 > 16) 
          {
            Serial.printf("Invalid Plot number");
            buttonCharacteristic->setValue("Invalid_Plot_number");
            buttonCharacteristic->notify();
          }
          else
            {
              //Completion time
              // Serial.printf("scan ");
              if(token!=NULL)
              {
                token = strtok(NULL,"-");
                CompleteIn = std::strtol(token, &ptr, 10);
                Serial.printf("Complete in:%ld\n",CompleteIn); 
                // Serial.printf("'%s'\n",token); 
               
                sprintf(str_msg,"S%c%c",(unsigned char) switchState1,(unsigned char)  CompleteIn);
                Serial2.write(str_msg);
                Serial.write(str_msg);
                // Sec = millis()/1000;
              }
            }
        }
 
      }
      else 
        if(res2 == 0)
        {
          Serial.printf("fetchNearest ");

          if(token!=NULL)
          {
            token = strtok(NULL,"-");
            
            res1 =  strcmp(token,cmd_1_1);
            res2 =  strcmp(token,cmd_1_2);

              //Completion time
              // Serial.printf("scan ");
            if(token!=NULL)
            {
              token = strtok(NULL,"-");
              CompleteIn = std::strtol(token, &ptr, 10);
              Serial.printf("Complete in:%ld\n",CompleteIn); 
              // Serial.printf("'%s'\n",token);           

              if(res1 == 0)
                {
                  Serial.printf("majorInjury\n");
                  sprintf(str_msg,"FM%c",(unsigned int) CompleteIn);
                  Serial2.write(str_msg);
                  Serial.write(str_msg);
                  // Serial2.write('1');
                  // Sec = millis()/1000;
                }
              else
                if(res2 == 0)
                {
                  Serial.printf("minorInjury\n");
                  sprintf(str_msg,"Fm%c",(unsigned int) CompleteIn);
                  Serial2.write(str_msg);
                  Serial.write(str_msg);
                  // Sec = millis()/1000;
                }
            }
          }
          else
          {
            Serial.printf("Invalid Injury\n");
            buttonCharacteristic->setValue("Invalid_Injury");
            buttonCharacteristic->notify();
          }
        }
        else
        {
          Serial.printf("Invalid Command received\n");
          // buttonCharacteristic->setValue("Invalid_Command_received");
          // buttonCharacteristic->notify();
        }
      // while(token!=NULL)
      // {
        // Serial.printf("'%s'\n",token);
        // token = strtok(NULL,"-");
      // }
      // Serial.printf("'%s'\n",token);
      // char *ptr;
      // long switchState1 = std::strtol(token, &ptr, 10);
      // Serial.printf("'%ld'\n",switchState1);
      // switchState1 = std::strtol(ptr, &ptr, 10);
      // Serial.printf("'%ld'\n",switchState1);
      // if (value.length() > 1) {
      //   Serial.println("Value should be 0 or 1");
      //   return;
      // }
      // long switchState = std::strtol(value.c_str(), NULL, 10);
      if (led_toggle) {
        digitalWrite(LED, LOW);
        led_toggle = 0;
      }
      else {
        digitalWrite(LED, HIGH);
        led_toggle = 1;
      }
    }
};


void setup() {
  //Serial 2 eneble
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));

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
  // lightSwitchService->start();


  TrackCharacteristic = lightSwitchService->createCharacteristic(
                          NOTIFY_TRACK_UUID,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  // client charactersitic descriptor: required for notify
  TrackCharacteristic->addDescriptor(new BLE2902());

  DebugCharacteristic = lightSwitchService->createCharacteristic(
                          NOTIFY_DEBUG_UUID,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  // client charactersitic descriptor: required for notify
  DebugCharacteristic->addDescriptor(new BLE2902());

  lightSwitchService->start();

  // off initially
  ledCharacteristic->setValue("0");
  buttonCharacteristic->setValue("0");
  TrackCharacteristic->setValue("0");
  DebugCharacteristic->setValue("0");

  // char testStr[] ="Request accepted";
  // TrackCharacteristic->setValue(testStr);
  // TrackCharacteristic->notify();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

int tem = 500;
int flag = 0;
char ch_msg;

void loop() { 

  if(deviceConnected == true) 
  {
  int index = 0;
  char testStr[] ="Request accepted------------------------------";
  // Serial2.write(str_msg);
  unsigned long time = millis();
  unsigned long delay_var = 3700;

  char Fwd_Trac[] = "forward-60-59";
  char scan_Trac[] = "scanned-5-green";

  // if(deviceConnected == false)  //Resets ESP32 if ble is diconnected
  //   ESP.restart();

  // while(1)
  // {
  //   if(millis() > (time+delay ) )
  //   {
  //     Serial.print(Fwd_Trac);
  //     TrackCharacteristic->setValue(Fwd_Trac);
  //     TrackCharacteristic->notify();

  //     // Serial.print(scan_Trac);
  //     // TrackCharacteristic->setValue(scan_Trac);
  //     // TrackCharacteristic->notify();

  //     time = millis();
  //   }

  // }
  while(Serial2.available())
  {
    
    ch_msg =char (Serial2.read());
    Serial.print(ch_msg);

    if(ch_msg=='^')
    {
      flag = 1;
      break;
    }
    else if(ch_msg == '!')
    {
      flag = 2;
      break;
    }
    else if(ch_msg == '$')
    {
      flag = 3;
      break;
    }

      if(index < 45)
        testStr[index++] = ch_msg;
      else  
        {
          Serial.print("\nError: Index excedded");
          Serial.print(testStr);
          break;
        }


    flag = 4;

      delay(2); //To enable reading of entrire string

  }

  if(flag==1)
  {

    testStr[index] = '\0';//NULL;
    Serial.print("\nCmd:");
    Serial.print(testStr);
      // buttonCharacteristic->setValue(d.c_str());
      buttonCharacteristic->setValue(testStr);
      buttonCharacteristic->notify();
    flag = 0;
  }
  else if(flag==2)
  {
    testStr[index] = '\0';//NULL;
    Serial.print("\nTrack:");
    Serial.print(testStr);
    // buttonCharacteristic->setValue(d.c_str());
    TrackCharacteristic->setValue(testStr);
    TrackCharacteristic->notify();
    flag = 0;
  }
  else if(flag==3)
  {
    testStr[index] = '\0';//NULL;
    Serial.print("\nDebug:");
    Serial.print(testStr);

    DebugCharacteristic->setValue(testStr);
    DebugCharacteristic->notify();
    flag = 0;
  }
  else if(flag==4)
  {
    Serial.print("\nGarbage data flushed");
    // Serial.print(testStr);
    index = 0;
  }

  }
}


void pin_ISR() {
  buttonState = digitalRead(BUTTON);
}
