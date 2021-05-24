#include <Arduino.h>

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define SERIAL_BAUD 57600        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
// #define DEBUG_RX                // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//only pick one taskrate for this simple test
#define TASK_1000MS 0 //simple task scheduling of 1 second
#define TASK_500MS 1
#define TASK_200MS 0 //simple task scheduling of 200ms
#define TASK_10MS 1
#define TASK_5MS 0

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

//rolling buffer to store last 50 samples
int16_t left_dc_curr_buf[50] = {0};
int16_t speed_left[50] = {0};
int16_t batVoltage_buf[50] = {0};
int16_t temp_board_buf[50] = {0};
int16_t iq_buf[50] = {0};
long timestamps_buff[50] = {0};
int buffer_index = 0; //rolling index

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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

// #define RXD2 16
// #define TXD2 17
void setup()
{
  // put your setup code here, to run once:

  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(SERIAL_BAUD); //this is printed to monitor through computer
  Serial.println("Hoverboard Serial test");
  Serial2.begin(HOVER_SERIAL_BAUD); //used to communicate with hoverboard
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
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
      iq_buf[buffer_index] = Feedback.cmd2;
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

// ########################## LOOP ##########################

void loop(void)
{

  Receive(); //should not be run in a task rate apparantly

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

    Send(0, 200); //Send(int16_t uSteer, int16_t uSpeed)
                  // Receive(); // Check for new received data
  }
  else if (micros() < b)
    b = 0;
#endif

#if TASK_200MS
  static long unsigned int c = micros();
  if (micros() >= (c + 200000))
  { //200 ms tasks
    c = micros();

    // Receive();     // Check for new received data
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
    Serial.print("  timestamp:");
    Serial.print(timestamps_buff[buffer_index]);
    Serial.print("  left_dc_current[cA]:");
    Serial.print(left_dc_curr_buf[buffer_index]);
    Serial.print("  batt_vlt:");
    Serial.print(batVoltage_buf[buffer_index]);
    Serial.print("  spd_act_lft[rpm]:");
    Serial.print(speed_left[buffer_index]);
    Serial.print("  iq[cA?]:");
    Serial.print(iq_buf[buffer_index]);
    Serial.print("  temper [deci Cels]:");
    Serial.println(temp_board_buf[buffer_index]);
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
    Receive();    // Check for new received data
    Send(0, 100); //Send(int16_t uSteer, int16_t uSpeed)
  }
  else if (micros() < g)
    g = 0;
#endif
}