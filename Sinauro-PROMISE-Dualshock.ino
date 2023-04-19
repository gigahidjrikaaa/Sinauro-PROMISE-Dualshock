#include <ezButton.h>
#include <PS4Controller.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove

#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

const int motorPinFwd[2] = {5, 13};
const int reversePin[2] = {12, 23};
const int resetFlashPin = 18;
const int resetESPin = 14;
const int buzzerPin = 19;
const int greenLight = 2;
const int yellowLight = 2;
const int redLight = 2;
bool revState[2] = {0, 0};
int speed[2] = {0, 0};
bool reset = 0;

ezButton resetButton(resetFlashPin);

// Interrupt service routine to reset the ESP32
void IRAM_ATTR resetModule() {
  Serial.println("Resetting module...");
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(200);
  }
  esp_restart();
}

void setup() {

  Serial.begin(115200);

  resetButton.setDebounceTime(50);

  // attach the interrupt to the reset button when it is pressed
  attachInterrupt(digitalPinToInterrupt(resetESPin), resetModule, FALLING);

  // Replace the "1a:2b:3c:01:01:01" with the MAC address of the laptop
  PS4.begin("50:84:92:81:b7:a7");
  Serial.println("Ready.");
  pinMode(buzzerPin, OUTPUT);
  for(int i = 0; i < 2; i++)
  {
    pinMode(motorPinFwd[i], OUTPUT);
    pinMode(reversePin[i], OUTPUT);
  }
  pinMode(redLight, OUTPUT);
  pinMode(greenLight, OUTPUT);
  pinMode(yellowLight, OUTPUT);
}

void loop() {
  resetButton.loop();
  if(resetButton.isPressed() && !reset)
  {
    Serial.println("=========== FLASH MEMORY RESET ===========");
    digitalWrite(buzzerPin, HIGH);
    removeFlashMemory();
    delay(2000);
    digitalWrite(buzzerPin, LOW);
    reset = true;
  } else if(!resetButton.isPressed() && reset)
  {
    reset = false;
  }

  if (PS4.isConnected()) {
    if (PS4.Right()) Serial.println("Right Button");
    if (PS4.Down()) Serial.println("Down Button");
    if (PS4.Up()) Serial.println("Up Button");
    if (PS4.Left()) Serial.println("Left Button");

    if (PS4.Square()) Serial.println("Square Button");
    if (PS4.Cross()){
      digitalWrite(buzzerPin, HIGH);
    } else{
      digitalWrite(buzzerPin, LOW);
    }
    if (PS4.Circle()) Serial.println("Circle Button");
    if (PS4.Triangle()) Serial.println("Triangle Button");

    if (PS4.UpRight()) Serial.println("Up Right");
    if (PS4.DownRight()) Serial.println("Down Right");
    if (PS4.UpLeft()) Serial.println("Up Left");
    if (PS4.DownLeft()) Serial.println("Down Left");

    if (PS4.L1())
    {
    }
    if (PS4.R1())
    {
    }

    if (PS4.Share()) Serial.println("Share Button");
    if (PS4.Options()) Serial.println("Options Button");
    if (PS4.L3()) Serial.println("L3 Button");
    if (PS4.R3()) Serial.println("R3 Button");

    if (PS4.PSButton()) ;
    if (PS4.Touchpad()) ;

    if (PS4.L2()) {
      
    }
    if (PS4.R2()) {
      
    }

    updateSpeed();
    printSpeed();
  }
  else {
    for(int i = 0; i < 2; i++)
    {
      speed[i] = 0;
    }
  }
  motorWrite();
}

void updateSpeed()
{
  for(int i = 0; i < 2; i++)
  {
    speed[i] = PS4.LStickY() * 1.5 + 0.25 * PS4.R2Value() - 0,25 * PS4.L2Value();
    if(i % 2 == 0)
      speed[i] -= PS4.RStickX() * 0.5 - PS4.Right() * 255 + PS4.Left() * 255 - PS4.Up() * 255 + PS4.Down() * 255;
    else if(i % 2 == 1)
      speed[i] += PS4.RStickX() * 0.5 - PS4.Right() * 255 + PS4.Left() * 255 + PS4.Up() * 255 - PS4.Down() * 255;

    if(speed[i] > 255)
      speed[i] = 255;
    else if(speed[i] < -255)
      speed[i] = -255;
  }
}
void removeFlashMemory()
{
  initBluetooth();
  Serial.print("ESP32 bluetooth address: "); Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    Serial.println("No bonded device found.");
  } else {
    Serial.print("Bonded device count: "); Serial.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      Serial.print("Reset bonded device count: "); Serial.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        Serial.print("Found bonded device # "); Serial.print(i); Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            Serial.print("Removed bonded device # "); 
          } else {
            Serial.print("Failed to remove bonded device # ");
          }
          Serial.println(i);
        }
      }        
    }
  }
}

unsigned long int printTime;
void printInfo()
{
  if(millis() - printTime >= 1000)
  {
    if (PS4.isConnected()) {
      if (PS4.Right()) Serial.println("Right Button");
      if (PS4.Down()) Serial.println("Down Button");
      if (PS4.Up()) Serial.println("Up Button");
      if (PS4.Left()) Serial.println("Left Button");

      if (PS4.Square()) Serial.println("Square Button");
      if (PS4.Cross()) Serial.println("Cross Button");
      if (PS4.Circle()) Serial.println("Circle Button");
      if (PS4.Triangle()) Serial.println("Triangle Button");

      if (PS4.UpRight()) Serial.println("Up Right");
      if (PS4.DownRight()) Serial.println("Down Right");
      if (PS4.UpLeft()) Serial.println("Up Left");
      if (PS4.DownLeft()) Serial.println("Down Left");

      if (PS4.L1()) Serial.println("L1 Button");
      if (PS4.R1()) Serial.println("R1 Button");

      if (PS4.Share()) Serial.println("Share Button");
      if (PS4.Options()) Serial.println("Options Button");
      if (PS4.L3()) Serial.println("L3 Button");
      if (PS4.R3()) Serial.println("R3 Button");

      if (PS4.PSButton()) Serial.println("PS Button");
      if (PS4.Touchpad()) Serial.println("Touch Pad Button");

      if (PS4.L2()) {
        Serial.printf("L2 button at %d\n", PS4.L2Value());
      }
      if (PS4.R2()) {
        Serial.printf("R2 button at %d\n", PS4.R2Value());
      }

      if (PS4.LStickX()) {
        Serial.printf("Left Stick x at %d\n", PS4.LStickX());
      }
      if (PS4.LStickY()) {
        Serial.printf("Left Stick y at %d\n", PS4.LStickY());
      }
      if (PS4.RStickX()) {
        Serial.printf("Right Stick x at %d\n", PS4.RStickX());
      }
      if (PS4.RStickY()) {
        Serial.printf("Right Stick y at %d\n", PS4.RStickY());
      }
      Serial.println();
    }
    printTime = millis();
  }
}

unsigned long int printSpeedTime;
void printSpeed()
{
  if(millis() - printSpeedTime >= 200)
  {
    Serial.printf("%d\tSpeed1: %d\tSpeed2: %d\tRev1: %d\tRev2: %d\n", millis(), speed[0], speed[1], revState[0], revState[1]);
    printSpeedTime = millis();
  } 
}

void motorWrite()
{
  for(int i = 0; i < 2; i++)
  {
    if(speed[i] < 0){
      digitalWrite(reversePin[i], HIGH);
    }
    else{
      digitalWrite(reversePin[i], LOW);
    }
    analogWrite(motorPinFwd[i], abs(speed[i]));
  }
}

bool initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

