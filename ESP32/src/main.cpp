// changelog
// 31 may 2021: added multiple BLE characteristics
// 01 jun 2021: added three BLE characteristics, added absolute value check to charge battery
// 6 jun 2021: trying to avoid time-outs
// 14 jun 2021: included bluetooth low energy over the air update from Felix Biego https://github.com/fbiego/ESP32_BLE_OTA_Arduino
// 26 jun 2021: added MQTT messages and OTA over wifi + together with 2nd ESP using https://github.com/martin-ger/esp32_nat_router

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h> //For MQTT
#include <WiFiUdp.h>      //For OTA
#include <ArduinoOTA.h>   //For OTA
#include <ESPmDNS.h>      //For OTA
// #include <esp_wifi.h>

#define SERIAL_BAUD 57600 // [-] Baud rate for built-in Serial (used for the Serial Monitor when connected to pc)
#define CONNECTED_TO_PC 1 // to enable/disable serial printout in majority of cases
#define BLUETOOTH 0       // to send BLE messages and allow OTA-BLE
#define WIFIMQTT 1        // send mqtt messages
#define SPEEDSIGN -1      //-1 or 1, 1 if the blades rotate at positive speed in the wind, -1 when negative

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define START_FRAME 0xABCD

//only pick one taskrate for this simple test
#define TASK_20000MS 1 //simple task scheduling of 20 second
#define TASK_1000MS 1  //simple task scheduling of 1 second
#define TASK_500MS 1
#define TASK_200MS 1 //simple task scheduling of 200ms
#define TASK_10MS 1
#define TASK_5MS 0

#if WIFIMQTT

#include "configuration.h" // to store passwords of mqtt and wifi

//MQTT client
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

//Necesary to make Arduino Software autodetect OTA device
WiFiServer TelnetServer(8266);

//mqtt
String mqtt_client_id = "HOVERWIND"; //This text is concatenated with ChipId to get unique client_id
String mqtt_base_topic = "/HOVERWIND";
#define spd_topic "/speed"
#define trq_topic "/torque"
#define volt_topic "/volt"
#define power_topic "/power"
#define counter_topic "/cntr"

#endif

#if BLUETOOTH
#include <Update.h>
#include "FS.h"
#include "FFat.h"
#include "SPIFFS.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLEServer *pServer;
BLEService *pService;
BLEAdvertising *pAdvertising;

BLECharacteristic *pCharacteristic1;
BLECharacteristic *pCharacteristic2;
BLECharacteristic *pCharacteristic3;
BLECharacteristic *pCharacteristic4;

static BLECharacteristic *pCharacteristicTX;
static BLECharacteristic *pCharacteristicRX;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  //power
#define CHARACTERISTIC2_UUID "688091db-1736-4179-b7ce-e42a724a6a68" //battery voltage
#define CHARACTERISTIC3_UUID "d240dbed-7d22-45bb-b810-add58a6c856b" // rpm
#define CHARACTERISTIC4_UUID "41e5e3f7-47e2-4885-945c-9dda1fc1dc7c" // operating state

#define SERVICE_UUID2 "fb1e4001-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_RX "fb1e4002-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_TX "fb1e4003-54ae-4a28-9f74-dfccb248601d"

// BLUETOOTH OTA definitions
// #define BUILTINLED 2
#define FORMAT_SPIFFS_IF_FAILED true
#define FORMAT_FFAT_IF_FAILED true
#define USE_SPIFFS //comment to use FFat

#ifdef USE_SPIFFS
#define FLASH SPIFFS
#define FASTMODE false //SPIFFS write is slow
#else
#define FLASH FFat
#define FASTMODE true //FFat is faster
#endif

#define NORMAL_MODE 0 // normal
#define UPDATE_MODE 1 // receiving firmware
#define OTA_MODE 2    // installing firmware

uint8_t updater[16384];
uint8_t updater2[16384];

static bool deviceConnected = false, sendMode = false;
static bool writeFile = false, request = false;
static int writeLen = 0, writeLen2 = 0;
static bool current = true;
static int parts = 0, next = 0, cur = 0, MTU = 0;
static int MODE = NORMAL_MODE;

/////////////////////////////////////////////////////////////////////////////////
////////////////////// FUNCTIONS BLE-OTA/////////////////////////////////////////

static void rebootEspWithReason(String reason)
{
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{

  //    void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) {
  //      Serial.print("Status ");
  //      Serial.print(s);
  //      Serial.print(" on characteristic ");
  //      Serial.print(pCharacteristic->getUUID().toString().c_str());
  //      Serial.print(" with code ");
  //      Serial.println(code);
  //    }

  void onNotify(BLECharacteristic *pCharacteristic)
  {
    uint8_t *pData;
    std::string value = pCharacteristic->getValue();
    int len = value.length();
    pData = pCharacteristic->getData();
    if (pData != NULL)
    {
      //        Serial.print("Notify callback for characteristic ");
      //        Serial.print(pCharacteristic->getUUID().toString().c_str());
      //        Serial.print(" of data length ");
      //        Serial.println(len);
      Serial.print("TX  ");
      for (int i = 0; i < len; i++)
      {
        Serial.printf("%02X ", pData[i]);
      }
      Serial.println();
    }
  }

  void onWrite(BLECharacteristic *pCharacteristic)
  {
    uint8_t *pData;
    std::string value = pCharacteristic->getValue();
    int len = value.length();
    pData = pCharacteristic->getData();
    if (pData != NULL)
    {
      //        Serial.print("Write callback for characteristic ");
      //        Serial.print(pCharacteristic->getUUID().toString().c_str());
      //        Serial.print(" of data length ");
      //        Serial.println(len);
      //        Serial.print("RX  ");
      //        for (int i = 0; i < len; i++) {         // leave this commented
      //          Serial.printf("%02X ", pData[i]);
      //        }
      //        Serial.println();

      if (pData[0] == 0xFB)
      {
        int pos = pData[1];
        for (int x = 0; x < len - 2; x++)
        {
          if (current)
          {
            updater[(pos * MTU) + x] = pData[x + 2];
          }
          else
          {
            updater2[(pos * MTU) + x] = pData[x + 2];
          }
        }
      }
      else if (pData[0] == 0xFC)
      {
        if (current)
        {
          writeLen = (pData[1] * 256) + pData[2];
        }
        else
        {
          writeLen2 = (pData[1] * 256) + pData[2];
        }
        current = !current;
        cur = (pData[3] * 256) + pData[4];
        writeFile = true;
        if (cur < parts - 1)
        {
          request = !FASTMODE;
        }
      }
      else if (pData[0] == 0xFD)
      {
        sendMode = true;
        if (FLASH.exists("/update.bin"))
        {
          FLASH.remove("/update.bin");
        }
      }
      else if (pData[0] == 0xFF)
      {
        parts = (pData[1] * 256) + pData[2];
        MTU = (pData[3] * 256) + pData[4];
        MODE = UPDATE_MODE;
      }
    }
  }
};

static void writeBinary(fs::FS &fs, const char *path, uint8_t *dat, int len)
{

  Serial.printf("Write binary file %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);

  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  file.write(dat, len);
  file.close();
}

void sendOtaResult(String result)
{
  pCharacteristicTX->setValue(result.c_str());
  pCharacteristicTX->notify();
  delay(200);
}

void performUpdate(Stream &updateSource, size_t updateSize)
{
  char s1 = 0x0F;
  String result = String(s1);
  if (Update.begin(updateSize))
  {
    size_t written = Update.writeStream(updateSource);
    if (written == updateSize)
    {
      Serial.println("Written : " + String(written) + " successfully");
    }
    else
    {
      Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
    }
    result += "Written : " + String(written) + "/" + String(updateSize) + " [" + String((written / updateSize) * 100) + "%] \n";
    if (Update.end())
    {
      Serial.println("OTA done!");
      result += "OTA Done: ";
      if (Update.isFinished())
      {
        Serial.println("Update successfully completed. Rebooting...");
        result += "Success!\n";
      }
      else
      {
        Serial.println("Update not finished? Something went wrong!");
        result += "Failed!\n";
      }
    }
    else
    {
      Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      result += "Error #: " + String(Update.getError());
    }
  }
  else
  {
    Serial.println("Not enough space to begin OTA");
    result += "Not enough space for OTA";
  }
  if (deviceConnected)
  {
    sendOtaResult(result);
    delay(5000);
  }
}

void updateFromFS(fs::FS &fs)
{
  File updateBin = fs.open("/update.bin");
  if (updateBin)
  {
    if (updateBin.isDirectory())
    {
      Serial.println("Error, update.bin is not a file");
      updateBin.close();
      return;
    }

    size_t updateSize = updateBin.size();

    if (updateSize > 0)
    {
      Serial.println("Trying to start update");
      performUpdate(updateBin, updateSize);
    }
    else
    {
      Serial.println("Error, file is empty");
    }

    updateBin.close();

    // when finished remove the binary from spiffs to indicate end of the process
    Serial.println("Removing update file");
    fs.remove("/update.bin");

    rebootEspWithReason("Rebooting to complete OTA update");
  }
  else
  {
    Serial.println("Could not load update.bin from spiffs root");
  }
}

void initBLE()
{
  BLEDevice::init("ESP32 Hover");
  // BLEServer *pServer = BLEDevice::createServer();
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService2 = pServer->createService(SERVICE_UUID2);
  // pService = pServer->createService(SERVICE_UUID2);
  pCharacteristicTX = pService2->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicRX = pService2->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRX->setCallbacks(new MyCallbacks());
  pCharacteristicTX->setCallbacks(new MyCallbacks());
  pCharacteristicTX->addDescriptor(new BLE2902());
  pCharacteristicTX->setNotifyProperty(true);
  pService2->start();

  //get other data over bluetooth
  // pService = pServer->createService(SERVICE_UUID2);
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic1 = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic1->setValue("Start1");

  pCharacteristic2 = pService->createCharacteristic(
      CHARACTERISTIC2_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic2->setValue("start2");

  pCharacteristic3 = pService->createCharacteristic(
      CHARACTERISTIC3_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic3->setValue("start3");

  pCharacteristic4 = pService->createCharacteristic(
      CHARACTERISTIC4_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic4->setValue("start4");
  pService->start();
  //  pAdvertising = pServer->getAdvertising();
  //  pAdvertising->addServiceUUID(pService->getUUID());
  //  pAdvertising->start();

  //BLEAdvertising *pAdvertising2 = pServer->getAdvertising(); // this still is working for backward compatibility
  BLEAdvertising *pAdvertising2 = BLEDevice::getAdvertising();
  //pAdvertising2->addServiceUUID(SERVICE_UUID);
  pAdvertising2->addServiceUUID(SERVICE_UUID2);
  pAdvertising2->setScanResponse(true);
  pAdvertising2->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising2->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

#if CONNECTED_TO_PC
  Serial.println("Characteristic defined! Now you can read it in your phone!");
  Serial.println("Value Characteristic defined");
#endif
}

void send_BLE(int16_t value, BLECharacteristic *pCharacteristic)
{
  char buffer[80];
  dtostrf(value, 1, 2, buffer);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify(); // send notification of change
}
#endif
/////////////////////////////////////////////////////////////////////////////////
////////////////////// FUNCTIONS and structs HOVER /////////////////////////////////////////
// Global variables hoverboard
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

float motorconstant = 30; // cA/Nm, ugess

//rolling buffer to store last 50 samples of measurements
int16_t left_dc_curr_buf[50] = {0};
int16_t speed_left[50] = {0};
int16_t batVoltage_buf[50] = {0};
int16_t temp_board_buf[50] = {0};
int16_t iq_buf[50] = {0};
long timestamps_buff[50] = {0};
int buffer_index = 0; //rolling index

// struct for sending hoverboard data
typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

// struct for receiving hoverboard data
typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

//structure for PI speed controller
struct PI_ctrl
{
  float Kp;
  float Ti;
  float mem1;
  float mem2;
  float output;
  float samplingtime;
};
struct PI_ctrl PI_speed;

struct ctrl_Motor
{
  int8_t Speedloop_enable;
  int16_t Speedloop_setp;
  int8_t Torqueloop_enable;
  int16_t Torqueloop_setp;
};
struct ctrl_Motor ctrl_Motor_state = {0, 0, 0, 0};

void Send(int16_t uSteer, int16_t uSpeed)
{
  // send commands to hoverboard over uart (unfortunately in an inconvenient structure of steer and speed)
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *)&Command, sizeof(Command));
}

#if WIFIMQTT
void mqtt_reconnect()
{

  // Loop until we're reconnected
  uint8_t cntr = 0;
  while (!mqtt_client.connected() && cntr < 5)
  {
    cntr++;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again");
      delay(500);
    }
  }
}

void setup_wifi()
{
  // delay(10);
  // Serial.print("Connecting to ");
  // Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  uint8_t cntr = 0;

  while (WiFi.status() != WL_CONNECTED && cntr < 25)
  {
    delay(500);
    Serial.println(cntr);
    cntr++;
  }
  Serial.println("OK");
  Serial.print("   IP address: ");
  Serial.println(WiFi.localIP());

  if (cntr < 25)
  {
    digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED)); //blink LED when connected to wifi
  }
}

#endif

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available())
  {
    incomingByte = Serial2.read();                                      // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.println(incomingByte);
  // Serial.println("bytes read");
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      // from main.c in hoverboard firmware
      // #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      //   if (main_loop_counter % 2 == 0) {    // Send data periodically every 10 ms
      //     Feedback.start	        = (uint16_t)SERIAL_START_FRAME;
      //     Feedback.cmd1           = (int16_t)input1[inIdx].cmd;
      //     Feedback.cmd2           = (int16_t)input2[inIdx].cmd;
      //     Feedback.speedR_meas	  = (int16_t)rtY_Right.n_mot;
      //     Feedback.speedL_meas	  = (int16_t)rtY_Left.n_mot;
      //     Feedback.batVoltage	    = (int16_t)batVoltageCalib;
      //     Feedback.boardTemp	    = (int16_t)board_temp_deg_c;

      // int16_t left_dc_curr;            // global variable for Left DC Link current
      /* Outport: '<Root>/iq' */
      // rtY->iq = rtDW->DataTypeConversion[0];

      /* Outport: '<Root>/id' */
      // rtY->id = rtDW->DataTypeConversion[1];
      //     #if defined(FEEDBACK_SERIAL_USART2)
      //       if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
      //         Feedback.cmdLed     = (uint16_t)sideboard_leds_L;
      //         Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas
      //                                        ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

      //         HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(Feedback));
      //       }
      //     #endif

      // Print data to built-in Serial
      // Serial.print("1: ");
      // Serial.print(Feedback.cmd1);
      // Serial.print(" 2: ");
      // Serial.print(Feedback.cmd2);
      // Serial.print(" 3: ");
      // Serial.print(Feedback.speedR_meas);
      // Serial.print(" 4: ");
      // Serial.print(Feedback.speedL_meas);
      // Serial.print(" 5: ");
      // Serial.print(Feedback.batVoltage);
      // Serial.print(" 6: ");
      // Serial.print(Feedback.boardTemp);
      // Serial.print(" 7: ");
      // Serial.println(Feedback.cmdLed);

      // store in buffer instead of fludding serial
      left_dc_curr_buf[buffer_index] = Feedback.speedR_meas;
      speed_left[buffer_index] = Feedback.speedL_meas;
      batVoltage_buf[buffer_index] = Feedback.batVoltage;
      temp_board_buf[buffer_index] = Feedback.boardTemp;
      // iq_buf[buffer_index] = Feedback.cmd2;
      iq_buf[buffer_index] = (int16_t)(Feedback.cmd2 >> 4);

      timestamps_buff[buffer_index] = millis();
      buffer_index = buffer_index + 1;

      if (buffer_index > 49)
      {
        buffer_index = 0;
      }
    }
    else
    {
      // Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

struct PI_ctrl PI_loop(struct PI_ctrl PI_in, float measurement, float setpoint)
{
  // PI control including an anti-windup

  //  struct PI_ctrl{
  //  float Kp;
  //  float Ti;
  //  float mem1;
  //  float mem2;
  //  float output;
  //  float samplingtime;
  //}

  struct PI_ctrl PI_out;
  float c0 = 0;
  float c1 = 0;
  float error = 0;
  float Ut1 = 0;
  float Et1 = 0;

  PI_out = PI_in;

  c0 = PI_in.Kp * (1 + PI_in.samplingtime / PI_in.Ti);
  c1 = -PI_in.Kp;

  Ut1 = PI_in.mem1;
  Et1 = PI_in.mem2;

  //anti-reset windup back clamping
  Et1 = Et1 + (PI_in.output - Ut1) / c0;
  Ut1 = PI_in.output;

  error = setpoint - measurement;
  PI_out.output = Ut1 + c0 * error + c1 * Et1;

  PI_out.mem1 = PI_out.output;
  PI_out.mem2 = error;

  return PI_out;
}

struct ctrl_Motor DisChargeBattery(float powerMotr_watt, ctrl_Motor ctrl_MotorIn)
{
  // test function to deplete battery and let turbine spin

  //  struct ctrl_Motor{
  //  int8_t Speedloop_enable;
  //  int16_t Speedloop_setp;
  //  int8_t Torqueloop_enable;
  //  int16_t Torqueloop_setp;
  //}
  struct ctrl_Motor ctrl_MtrOut;

  ctrl_MtrOut.Speedloop_setp = abs(ctrl_MotorIn.Speedloop_setp);
  ctrl_MtrOut.Speedloop_enable = 1;
  ctrl_MtrOut.Torqueloop_enable = 0;
  ctrl_MtrOut.Torqueloop_setp = 0;

  if (powerMotr_watt < -5)
  {
    ctrl_MtrOut.Speedloop_setp = ctrl_MtrOut.Speedloop_setp + 5;
  }

  if (ctrl_MtrOut.Speedloop_setp < 100)
  {
    ctrl_MtrOut.Speedloop_setp = 100;
  }
  else if (ctrl_MtrOut.Speedloop_setp > 500)
  {
    ctrl_MtrOut.Speedloop_setp = 500;
  }

  ctrl_MtrOut.Speedloop_setp = SPEEDSIGN * ctrl_MtrOut.Speedloop_setp;

  return ctrl_MtrOut;
}

struct ctrl_Motor ChargeBattery(float powerMotr_watt, int16_t speed_mtr, float command_trq, ctrl_Motor ctrl_MotorIn)
{
  // very simplistic speed control to charge battery
  struct ctrl_Motor ctrl_MtrOut;
  ctrl_MtrOut = ctrl_MotorIn;

  // torque command is low and motor is rotating --> turbine is generating and overcoming the "electric brake" torque
  // increase speed to get to a more efficient zone
  // needs some additional filtering instead of instant values
  if ((abs(speed_mtr) > 35) && (abs(command_trq) < 25))
  {
    ctrl_MtrOut.Speedloop_enable = 1;
    ctrl_MtrOut.Speedloop_setp = 125;
    ctrl_MtrOut.Torqueloop_enable = 0;
    ctrl_MtrOut.Torqueloop_setp = 0;
  }

  //wind got down
  if (powerMotr_watt > 0)
  {
    // if power gets positive, it's not generating power --> decrease speed setpoint
    ctrl_MtrOut.Speedloop_enable = 1;
    ctrl_MtrOut.Speedloop_setp = abs(ctrl_MotorIn.Speedloop_setp) - 1;
    ctrl_MtrOut.Torqueloop_enable = 0;
    ctrl_MtrOut.Torqueloop_setp = 0;

    if (ctrl_MtrOut.Speedloop_setp < 30)
    {
      // if speed needs to get too low, just let the automatic brake torque from the hoverboard firmware kick in
      // (parameters ELECTRIC_BRAKE_ENABLE, ELECTRIC_BRAKE_MAX,ELECTRIC_BRAKE_THRES)
      ctrl_MtrOut.Speedloop_enable = 0;
      ctrl_MtrOut.Speedloop_setp = 0;
      ctrl_MtrOut.Torqueloop_enable = 1;
      ctrl_MtrOut.Torqueloop_setp = 0;
    }

    ctrl_MtrOut.Speedloop_setp = SPEEDSIGN * ctrl_MtrOut.Speedloop_setp;
  }

  return ctrl_MtrOut;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////// execution

void setup()
{
  // setCpuFrequencyMhz(80);           //powering from the 5v of the hall sensors, reducing clock frequency hoping not to exceed 100mA
  Serial2.begin(HOVER_SERIAL_BAUD); //used to communicate with hoverboard
  pinMode(LED_BUILTIN, OUTPUT);

#if CONNECTED_TO_PC
  Serial.begin(SERIAL_BAUD);
  Serial.println("Starting hoverturbine ketch");
#endif

  // PI controller for voltage regulation
  PI_speed.Kp = 0.3;
  PI_speed.Ti = 0.4;
  PI_speed.mem1 = 0;
  PI_speed.mem2 = 0;
  PI_speed.output = 0;
  PI_speed.samplingtime = 0.2;

  // ESP_BT.begin("ESP32_Wind"); //Name of your Bluetooth Signal
#if BLUETOOTH

#ifdef USE_SPIFFS
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
#else
  if (!FFat.begin())
  {
    Serial.println("FFat Mount Failed");
    if (FORMAT_FFAT_IF_FAILED)
      FFat.format();
    return;
  }
#endif

  initBLE();
#endif

#if WIFIMQTT

  setup_wifi();

  //OTA things
  Serial.print("Configuring OTA device...");
  TelnetServer.begin(); //Necesary to make Arduino Software autodetect OTA device
  ArduinoOTA.onStart([]()
                     { Serial.println("OTA starting..."); });
  ArduinoOTA.onEnd([]()
                   {
                     Serial.println("OTA update finished!");
                     Serial.println("Rebooting...");
                   });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("OTA in progress: %u%%\r\n", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                     });
  ArduinoOTA.begin();
  Serial.println("OTA OK");

  mqtt_client.setServer(mqtt_server, 1883);
#if CONNECTED_TO_PC
  Serial.println("Configuring MQTT server...");
  Serial.printf("   Server IP: %s\r\n", mqtt_server);
  Serial.printf("   Username:  %s\r\n", mqtt_user);
  // Serial.println("   Cliend Id: " + mqtt_client_id);
  Serial.println("   MQTT configured!");
  Serial.println("Setup completed! Running app...");
#endif
#endif
}

////////////////////////////////////////////////////////////////////////////////
// repetitive tasks
////////////////////////////////////////////////////////////////////////////////
uint32_t timeout_avoid_counter = 0;
int16_t motor_setp = 0; // (torque related setpoint) to firmware of hoverboard
float power1 = 0;       //actual power of hoverboard

uint16_t profile_counter = 0;
uint16_t message_counter = 0;

enum charging_state
{
  ChargeBat,
  DischargeBat,
  TorqueFree
};
enum charging_state charging_state;

void loop()
{

  ArduinoOTA.handle();

#if BLUETOOTH

  switch (MODE)
  {

  case NORMAL_MODE:
    if (deviceConnected)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      if (sendMode)
      {
        uint8_t fMode[] = {0xAA, FASTMODE};
        pCharacteristicTX->setValue(fMode, 2);
        pCharacteristicTX->notify();
        delay(50);
        sendMode = false;
        Serial.print('.');
      }

      // your loop code here
    }

    else
    {
      digitalWrite(LED_BUILTIN, LOW);
    }

    // or here
#endif
    Receive(); //receiving hoverboard data should not be run in a task rate apparantly

#if TASK_5MS
    static long unsigned int a = micros();
    if (micros() >= (a + 5000))
    { //5 ms tasks
      a = micros();
      /* Tasklist: 5 ms */
      Receive(); // Check for new received data
    }
    else if (micros() < a)
      a = 0;
#endif

#if TASK_10MS
    static long unsigned int b = micros();
    if (micros() >= (b + 10000))
    { //10 ms tasks
      b = micros();
      /* Tasklist: 10 ms */

      // avoid timeout of firmware hoverboard
      if (abs(motor_setp) < 50)
      {
        timeout_avoid_counter++;
      }
      else
      {
        timeout_avoid_counter = 0;
      }

      if (timeout_avoid_counter > 30000)
      {
        motor_setp = SPEEDSIGN * 101;
        timeout_avoid_counter = 0;
      }

      // spin a bit when starting up, this is test code
      if (timeout_avoid_counter < 1500)
      {
        motor_setp = SPEEDSIGN * 101;
      }

      // //send serial commands to hoverboard
      // Send(0, motor_setp); //Send(int16_t uSteer, int16_t uSpeed)
    }
    else if (micros() < b)
      b = 0;
#endif

#if TASK_200MS
    static long unsigned int c = micros();
    if (micros() >= (c + 200000))
    { //200 ms tasks
      c = micros();

      if (ctrl_Motor_state.Torqueloop_enable == 1)
      {
        motor_setp = ctrl_Motor_state.Torqueloop_setp;
      }

      if (ctrl_Motor_state.Speedloop_enable == 1)
      {
        PI_speed = PI_loop(PI_speed, (float)(speed_left[buffer_index]), (float)(ctrl_Motor_state.Speedloop_setp));

        // put some limits on output
        if (PI_speed.output > 1000)
        {
          PI_speed.output = 1000;
        }
        else if (PI_speed.output < -1000)
        {
          PI_speed.output = -1000;
        }
        motor_setp = (int16_t)(PI_speed.output);
      }
    }
    else if (micros() < c)
      c = 0;
#endif

#if TASK_500MS
    static long unsigned int f = micros();
    if (micros() >= (f + 500000))
    { //500 ms tasks
      f = micros();
      /* Tasklist: 500 ms */
      // only print first element of buffer
    }
    else if (micros() < f)
      f = 0;
#endif

#if TASK_1000MS
    static long unsigned int g = micros();
    if (micros() >= (g + 1000000))
    { //1000 ms tasks
      g = micros();
      /* Tasklist: 1000 ms */

      power1 = 3.1415 / 30 * (float)(speed_left[buffer_index]) * (float)(iq_buf[buffer_index]) / motorconstant;

      long time_since_last_update = abs(millis() - timestamps_buff[buffer_index]);
      if (time_since_last_update > 5000)
      {
        charging_state = TorqueFree;
      }

      // desired state of windturbine
      // if ((batVoltage_buf[buffer_index] > 4150) && (time_since_last_update < 5000))
      // {
      //   charging_state = DischargeBat;
      // }
      if ((batVoltage_buf[buffer_index] <= 4250) && (time_since_last_update < 5000))
      {
        charging_state = ChargeBat;
      }
      // else
      // {
      //   charging_state = TorqueFree;
      // }

      // actions related to desired state
      if (charging_state == DischargeBat)
      {
        ctrl_Motor_state = DisChargeBattery(power1, ctrl_Motor_state);
      }
      else if (charging_state == ChargeBat)
      {

        ctrl_Motor_state = ChargeBattery(power1, speed_left[buffer_index], PI_speed.output, ctrl_Motor_state);
      }

      if (charging_state == TorqueFree)
      {
        ctrl_Motor_state.Torqueloop_enable = 1;
        ctrl_Motor_state.Speedloop_enable = 0;
        ctrl_Motor_state.Torqueloop_setp = 0;
        ctrl_Motor_state.Speedloop_setp = 0;
      }
    }
    else if (micros() < g)
      g = 0;
#endif

#if TASK_20000MS
    static long unsigned int h = micros();
    if (micros() >= (h + 20000000))
    { //20000 ms tasks
      h = micros();

#if WIFIMQTT
      if (!mqtt_client.connected())
      {
        setup_wifi();
        mqtt_reconnect();
      }

      // Serial.println("mqtt crap");
      // Serial.println(motor_setp);

      message_counter = message_counter + 1;
      if (message_counter > 65533)
      {
        message_counter = 0;
      }

      mqtt_client.publish((mqtt_base_topic + spd_topic).c_str(), String((float_t)(speed_left[buffer_index]), 2).c_str(), true);
      mqtt_client.publish((mqtt_base_topic + trq_topic).c_str(), String((float_t)motor_setp, 2).c_str(), true);
      mqtt_client.publish((mqtt_base_topic + volt_topic).c_str(), String((float_t)(batVoltage_buf[buffer_index]), 2).c_str(), true);
      mqtt_client.publish((mqtt_base_topic + power_topic).c_str(), String((float_t)(power1), 2).c_str(), true);
      mqtt_client.publish((mqtt_base_topic + counter_topic).c_str(), String((float_t)(message_counter), 2).c_str(), true);

      // digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED)); //blink LED when connected

#endif

#if BLUETOOTH
#if CONNECTED_TO_PC
      Serial.print("update BLE values ");
#endif

      // Serial.println(timeout_avoid_counter);
      send_BLE((int16_t)(power1), pCharacteristic1);
      send_BLE((int16_t)(batVoltage_buf[buffer_index]), pCharacteristic2);
      send_BLE((int16_t)(speed_left[buffer_index]), pCharacteristic3);
      send_BLE((int16_t)(charging_state), pCharacteristic4);

#endif
    }
    else if (micros() < h)
      h = 0;

#endif

#if BLUETOOTH

    break;

  case UPDATE_MODE:

    if (request)
    {
      uint8_t rq[] = {0xF1, (cur + 1) / 256, (cur + 1) % 256};
      pCharacteristicTX->setValue(rq, 3);
      pCharacteristicTX->notify();
      delay(50);
      request = false;
    }

    if (cur + 1 == parts)
    { // received complete file
      uint8_t com[] = {0xF2, (cur + 1) / 256, (cur + 1) % 256};
      pCharacteristicTX->setValue(com, 3);
      pCharacteristicTX->notify();
      delay(50);
      MODE = OTA_MODE;
    }

    if (writeFile)
    {
      if (!current)
      {
        writeBinary(FLASH, "/update.bin", updater, writeLen);
      }
      else
      {
        writeBinary(FLASH, "/update.bin", updater2, writeLen2);
      }
      writeFile = false;
    }

    break;

  case OTA_MODE:
    // Serial.println("ota mode");
    updateFromFS(FLASH);
    break;
  }

#endif
}
