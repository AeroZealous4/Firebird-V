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

#define RXD2 16
#define TXD2 17
char str_msg[] = "ESP string: Info will be appended on this string";
unsigned long int Sec = 0 ;

unsigned char led_toggle = 0;

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
      strcpy(str_msg,value.c_str());
      char *token = strtok(str_msg,"-");
      
      // char *cmd;
      char cmd_1[] = "scan";
      char cmd_2[] = "fetchNearest";
      char cmd_1_1[] = "majorInjury";
      char cmd_1_2[] = "minorInjury";

      // cmd = token;
      int res1 =  strcmp(token,cmd_1);
      int res2 =  strcmp(token,cmd_2);

      Serial.printf("'%s'\n",token);
      if(res1 == 0)
      {
        Serial.printf("scan ");
        token = strtok(NULL,"-");
        char *ptr;
        long switchState1 = std::strtol(token, &ptr, 10);
        Serial.printf("%ld\n",switchState1); 
        Serial.printf("'%s'\n",token);     

        if(switchState1 < 1 || switchState1 > 16) 
        {
          Serial.printf("Invalid Plot number");
          buttonCharacteristic->setValue("Invalid_Plot_number");
          buttonCharacteristic->notify();
        }
        else
          {
            sprintf(str_msg,"S%c",(unsigned char) switchState1);
            Serial2.write(str_msg);
            Sec = millis()/1000;
          }
      }
      else 
        if(res2 == 0)
        {
          Serial.printf("fetchNearest ");
          token = strtok(NULL,"-");
          
          res1 =  strcmp(token,cmd_1_1);
          res2 =  strcmp(token,cmd_1_2);

          if(res1 == 0)
            {
              Serial.printf("majorInjury\n");
              sprintf(str_msg,"FM");
              Serial2.write(str_msg);
              // Serial2.write('1');
              Sec = millis()/1000;
            }
          else
            if(res2 == 0)
            {
              Serial.printf("minorInjury\n");
              sprintf(str_msg,"Fm");
              Serial2.write(str_msg);
              Sec = millis()/1000;
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
          buttonCharacteristic->setValue("Invalid_Command_received");
          buttonCharacteristic->notify();
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

int tem =500;
int flag =0;
char ch_msg;
void loop() {
  int index = 0;
  String d = "";
  char testStr[] ="Request accepted------------------------------";
  // Serial2.write(str_msg);
  while(Serial2.available())
  {
    
    ch_msg =char (Serial2.read());
    Serial.print(ch_msg);

    if(ch_msg == 'z') //1
    {
      // testStr[] = "Request accepted";
      Serial.print("Request accepted");
      buttonCharacteristic->setValue("Request_accepted");
      buttonCharacteristic->notify();
    }        
    else if(ch_msg == 'q')  //2
    {
      Serial.print("Task Complete");
      sprintf(testStr,"%d",(int) (-Sec + millis()/1000) );
      buttonCharacteristic->setValue(testStr);
      buttonCharacteristic->notify();
    }        
    else if(ch_msg == '3')
    {
      Serial.print("Task Flushed");
      // sprintf(testStr,"%d",(int) (-Sec + millis()/1000) );
      buttonCharacteristic->setValue("Task_Flushed");
      buttonCharacteristic->notify();
    //  Serial.print("Task Not accepted or rejected");
    //  buttonCharacteristic->setValue("Task Not accepted or rejected");        
    }
    else 
    {
        d += ch_msg;
        testStr[index++] = ch_msg;
        flag = 1;
    }
      

  }
  if(flag)
  {
    d += '\0';//NULL;
    testStr[index] = '\0';//NULL;
      // buttonCharacteristic->setValue(d.c_str());
      buttonCharacteristic->setValue(testStr);
      buttonCharacteristic->notify();
    flag = 0;
  }
  // else
  //   Serial.print("Serial Port 2 not available");

  // delay(1000);
  // if (buttonState == LOW) {
  //   Serial.println("Button pressed!");
  //   if (buttonCharacteristic->getValue() == "0") {
  //     // Serial.println("Button pressed!");
  //     tem = (int) 0x0500;
  //     buttonCharacteristic->setValue(tem);
  //     buttonCharacteristic->notify();
  //   }
  //   else {
  //     tem = (int) 0x0700;
  //     buttonCharacteristic->setValue(tem);
  //     buttonCharacteristic->notify();
  //   }
  // }
}


void pin_ISR() {
  buttonState = digitalRead(BUTTON);
}
