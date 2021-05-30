#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
// #include "BluetoothSerial.h"

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define SERIAL_BAUD 57600        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
// #define DEBUG_RX                // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//only pick one taskrate for this simple test
#define TASK_1000MS 1 //simple task scheduling of 1 second
#define TASK_500MS 1
#define TASK_200MS 1 //simple task scheduling of 200ms
#define TASK_10MS 1
#define TASK_5MS 0

#define CONNECTED_TO_PC 0 // to enable/disable serial printout
#define SPEEDLOOP_ESP 1

// BLE is much more cumbersome versus bluetoothserial :(
// install a BLE scanner on your phone and look for the UUID to see a value getting updated on request ...

// BluetoothSerial ESP_BT; //Object for Bluetooth
BLECharacteristic *pCharacteristic;
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

float motorconstant = 30; // Nm/cA

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

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

/////////////////////////////////////////////////////////////////////////////////
////////////////////// FUNCTIONS /////////////////////////////////////////

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

  ctrl_MtrOut.Speedloop_setp = ctrl_MotorIn.Speedloop_setp;
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

  return ctrl_MtrOut;
}

struct ctrl_Motor ChargeBattery(float powerMotr_watt, int16_t speed_mtr, float command_trq, ctrl_Motor ctrl_MotorIn)
{
  // very simplistic speed control to charge battery
  struct ctrl_Motor ctrl_MtrOut;
  ctrl_MtrOut = ctrl_MotorIn;

  // torque command is low and motor is rotating --> turbine is generating and overcoming the "electric brake" torque
  // increase speed to get to a more efficient zone
  if ((speed_mtr > 60) && (command_trq < 25))
  {
    ctrl_MtrOut.Speedloop_enable = 1;
    ctrl_MtrOut.Speedloop_setp = 200;
    ctrl_MtrOut.Torqueloop_enable = 0;
    ctrl_MtrOut.Torqueloop_setp = 0;
  }

  //wind got down
  if (powerMotr_watt > 0)
  {
    // if power gets positive, it's not generating power --> decrease speed setpoint
    ctrl_MtrOut.Speedloop_enable = 1;
    ctrl_MtrOut.Speedloop_setp = ctrl_MotorIn.Speedloop_setp - 1;
    ctrl_MtrOut.Torqueloop_enable = 0;
    ctrl_MtrOut.Torqueloop_setp = 0;

    if (ctrl_MtrOut.Speedloop_setp < 50)
    {
      // if speed needs to get too low, just let the automatic brake torque from the hoverboard firmware kick in
      // (parameters ELECTRIC_BRAKE_ENABLE, ELECTRIC_BRAKE_MAX,ELECTRIC_BRAKE_THRES)
      ctrl_MtrOut.Speedloop_enable = 0;
      ctrl_MtrOut.Speedloop_setp = 0;
      ctrl_MtrOut.Torqueloop_enable = 1;
      ctrl_MtrOut.Torqueloop_setp = 0;
    }
  }

  return ctrl_MtrOut;
}

void send_BLE(int16_t value)
{
  char buffer[80];
  dtostrf(value, 1, 2, buffer);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify(); // send notification of change
}

//////////////////////////////////////////////////////////////////////////////////////////
// ########################## repeating loops ##########################
//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // put your setup code here, run once at startup:

  setCpuFrequencyMhz(80); //powering from the 5v of the hall sensors, reducing clock frequency hoping not to exceed 100mA

  pinMode(BUILTIN_LED, OUTPUT);
#if CONNECTED_TO_PC
  Serial.begin(SERIAL_BAUD); //this is printed to monitor through computer
  Serial.println("Hoverboard Serial test");
#endif

  Serial2.begin(HOVER_SERIAL_BAUD); //used to communicate with hoverboard

#if SPEEDLOOP_ESP
      // PI controller for voltage regulation
  PI_speed.Kp = 0.3;
  PI_speed.Ti = 0.4;
  PI_speed.mem1 = 0;
  PI_speed.mem2 = 0;
  PI_speed.output = 0;
  PI_speed.samplingtime = 0.2;
#endif

  // ESP_BT.begin("ESP32_Wind"); //Name of your Bluetooth Signal

  BLEDevice::init("ESP32_hover");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
}

int16_t motor_setp = 0; // (torque related setpoint) to firmware of hoverboard
float power1 = 0;       //actual power of hoverboard

uint16_t profile_counter = 0;

enum charging_state
{
  ChargeBat,
  DischargeBat,
  TorqueFree
};
enum charging_state charging_state;

// repetitive tasks

void loop(void)
{

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

    Send(0, motor_setp); //Send(int16_t uSteer, int16_t uSpeed)
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

#if SPEEDLOOP_ESP
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
#endif
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
    power1 = 3.1415 / 30 * (float)(speed_left[buffer_index]) * (float)(iq_buf[buffer_index]) / motorconstant;
    float power2 = 0.01 * (float)(batVoltage_buf[buffer_index]) * 0.01 * (float)(left_dc_curr_buf[buffer_index]);

#if CONNECTED_TO_PC
    Serial.print("  timestamp: ");
    Serial.print(timestamps_buff[buffer_index]);
    Serial.print("  command: ");
    Serial.print(motor_setp);
    Serial.print("  speed_setp  ");
    Serial.print(ctrl_Motor_state.Speedloop_setp);
    Serial.print("  speed enbl  ");
    Serial.print(ctrl_Motor_state.Speedloop_enable);
    Serial.print("  spd_act[rpm]: ");
    Serial.print(speed_left[buffer_index]);
    Serial.print("  dc_curr[cA]: ");
    Serial.print(left_dc_curr_buf[buffer_index]);
    Serial.print("  batt_vlt[cV]: ");
    Serial.print(batVoltage_buf[buffer_index]);
    Serial.print("  iq[cA?]: ");
    Serial.print(iq_buf[buffer_index]);
    Serial.print("  temper [deci Cels]: ");
    Serial.print(temp_board_buf[buffer_index]);

    Serial.print(" power1: ");
    // float power1 = 3.1415 / 30 * (float)(speed_left[buffer_index]) * (float)(iq_buf[buffer_index]) / motorconstant;
    Serial.print(power1);

    Serial.print("  power2: ");
    // float power2 = 0.01 * (float)(batVoltage_buf[buffer_index]) * 0.01 * (float)(left_dc_curr_buf[buffer_index]);
    Serial.print(power2);

    Serial.print("  state:  ");
    Serial.println(charging_state);
#endif

    // bluetooth non LE
    // if (ESP_BT.available()) //Check if we receive anything from Bluetooth
    // Serial.println("bluetooth start");
    // {
    // int incoming = ESP_BT.read(); //Read what we recevive
    // if (incoming == 49)
    // {
    //   digitalWrite(LED_BUILTIN, HIGH);
    //   ESP_BT.println("LED turned ON");
    // }

    // if (incoming == 48)
    // {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   ESP_BT.println("LED turned OFF");
    // }

    // ESP_BT.begin("  timestamp: ");
    // ESP_BT.begin(timestamps_buff[buffer_index]);
    // ESP_BT.begin("  command: ");
    // ESP_BT.begin(motor_setp);
    // ESP_BT.begin("  speed_setp  ");
    // ESP_BT.begin(ctrl_Motor_state.Speedloop_setp);
    // ESP_BT.begin("  speed enbl  ");
    // ESP_BT.begin(ctrl_Motor_state.Speedloop_enable);
    // ESP_BT.begin("  dc_curr[cA]: ");
    // ESP_BT.begin(left_dc_curr_buf[buffer_index]);
    // ESP_BT.begin("  batt_vlt[cV]: ");
    // ESP_BT.begin(batVoltage_buf[buffer_index]);
    // ESP_BT.begin("  spd_act[rpm]: ");
    // ESP_BT.begin(speed_left[buffer_index]);
    // ESP_BT.begin("  iq[cA?]: ");
    // ESP_BT.begin(iq_buf[buffer_index]);
    // ESP_BT.begin("  temper [deci Cels]: ");
    // ESP_BT.begin(temp_board_buf[buffer_index]);
    // ESP_BT.begin(" power1: ");
    // // float power1 = 3.1415 / 30 * (float)(speed_left[buffer_index]) * (float)(iq_buf[buffer_index]) / motorconstant;
    // ESP_BT.begin(power1);
    // ESP_BT.begin("  power2: ");
    // // float power2 = 0.01 * (float)(batVoltage_buf[buffer_index]) * 0.01 * (float)(left_dc_curr_buf[buffer_index]);
    // ESP_BT.begin(power2);
    // ESP_BT.begin("  state:  ");
    // ESP_BT.begin(charging_state);

    // Serial.println("bluetooth end");
    // }

    // if (profile_counter < 200)
    // {
    //   motor_setp = motor_setp + 1;
    // }
    // // else if ((profile_counter > 20) && (profile_counter < 40))
    // // {
    // //   motor_setp = motor_setp + 10;
    // // }
    // else
    // {
    //   motor_setp = 0;
    // }

    // profile_counter = profile_counter + 1;
    // if (profile_counter > 300)
    // {
    //   profile_counter = 0;
    // }

    // for (int i = 0; i < 50; i++)
    // {
    //   Serial.print("  timestamp:");
    //   Serial.print(timestamps_buff[i]);
    //   Serial.print("  meas:");
    //   Serial.println(speed_right[i]);
    // }

    // Receive();    // Check for new received data
    // Send(0, 100); //Send(int16_t uSteer, int16_t uSpeed)
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
    send_BLE((int16_t)(power1));

    long time_since_last_update = abs(millis() - timestamps_buff[buffer_index]);
    if (time_since_last_update > 5000)
    {
      charging_state = TorqueFree;
    }

    // desired state of windturbine
    if ((batVoltage_buf[buffer_index] > 4150) && (time_since_last_update < 5000))
    {
      charging_state = DischargeBat;
    }
    else if ((batVoltage_buf[buffer_index] < 4100) && (time_since_last_update < 5000))
    {
      charging_state = ChargeBat;
    }

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
}
