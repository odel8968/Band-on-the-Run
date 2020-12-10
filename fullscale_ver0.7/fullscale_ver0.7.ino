/*************************************************
   Band on the Run
   20 November 2020

   fullscale
   Version 0.7

   By Benjamin Odell

   Third Party Libraries Used:
   ESP32 PS4 Controller https://github.com/NURobotics/PS4-esp32

   ######## IMPORTANT NOTES ########
   -Calibrate RC Controller: Enter calibration mode once the RC controller
      is connected to the receiver by setting front
      right switch (SW A) to on (1). Move both sticks through their
      full range of motion, reaching the maximum positive and negative
      x and y values for both sticks. Once this has been completed,
      set switch back to off to resume normal operation.

   -Arduino IDE seems to have some issue with ESP32 bluetooth,
      after uploading if PS4 Controller stops connecting reflashing
      the bootloader fixes it. (I used the official esp-idf)

   -WiFi access point stops working when bluetooth is connected.
      Seems to be a limitation of the PS4 controller library, no plans on correcting.
   #################################
   Versions
   ~~MiniMe~~
   1.0
   Currently working with PS4 controller as tested with oscilloscope,
   RC receiver functionality implemented but not tested yet.

   1.1
   Fixed issue with ISR being called while in ISR,
   updated readController functions to pass inputs instead of wheel speed,
   added RC receiver calibration function,
   reconfigured receive channel declarations as arrays for easier expandability,
   revised mechanism for switching between controllers, now done with single #define,
   cleaned up lots of redundancies using loops.

   1.2
   Moved checking for controller timeout to readController() functions,
   added serial print debug statements enabled by #define var

   1.3
   Changed to task based scheduling for multitasking,
   added wireless access point functionality,
   verified RC controller functionality.

   1.4
   Changed to allow controller switching on the fly.

   ~~Fullscale~~
   0.1
   Added captive portal to the webpage, wrote rough rotary encoder ISR.

   0.2
   Added ILI9341 TFT display capability, RS485 working, Modbus ASCII
   interface in development.

   0.3
   Lots of cleanup and comments. Added software defined output ramp.
   Split RS485 communication into separate task. Limited scope of objects to their task when possible.
   PS4 Controller support commented out due to exceeding maximum program memory.
   Adjustment of speed limit and ramp time through captive portal added.


   ~~MiniMe~~
   2.0
   Converted new fullscale design to the work on the miniMe.

   2.1
   Added wheel speed measurement code and PID control.

   ~~Fullscale~~
   0.4
   Ported changes from miniMe back to fullscale.

   0.5
   Added MPU-9250 support for heading determination. Added changing
   of PID constants and toggling of PID control through webpage.
   Added IO Expander support and I2C Semaphore.

   0.6
   Changed output to use second ESP32 instead of IO Expander due to reliability.

*/

//#include <PS4Controller.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_ILI9341.h>
#include <PID_v1.h>
#include "ESP32MPU9250Madgwick.h"

#define PS4ControllerMAC "00:15:83:F2:82:BF"

#define printDebugStatements //Leave uncommented for serial debugging messages
const int printInterval = 2000;

/*~~~ Access point setup ~~~*/
const char* ssid     = "Band on the Run";
const char* password = "thisisatest";
const byte DNS_PORT = 53;

/*~~~ Choose Controller ~~~*/
enum controller {Playstation, RC};
int controllerSelected = RC;

/*********Pin Assignments*********/
/*********************************/
//const int directionPin[] = {8, 0, 0, 0}; //Define pins for motor direction control
//const int speedPin[] = {9, 0, 0, 0};  //Define pins for motor speed control
//const int receiveCh[] = {36, 39, 34, 35, 32, 33}; //pin assignment for the RC Receiver
const int receiveCh[] = {36, 39, 34, 35, 32}; //pin assignment for the RC Receiver
//const int screenPins[] = {12, 27, 26, 25, 14, 0}; //pin assignment for TFT display
const int encoderCh[] = {25, 26, 27, 14, 21, 19, 17, 16}; //Pins for rotary encoder
const int I2CPins[] = {22, 23}; //pins for I2C to read IMU values
const int IOExpanderPins[] = {5, 18};
/*************************************/
/*********End Pin Assignments*********/

/*~~~~~~~~ Define enums to make wheel and input array values easier to read ~~~~~~~~*/
enum wheel {Front, Right, Back, Left};
enum input {lxStick, lyStick, rxStick, ryStick, timeout};

/*~~~~~~~~ Define Variables for PWM channels ~~~~~~~~
const int pwmCh[] = {0, 1, 2, 3};     //Define channels for internal ESP32 PWM generator
const int freq = 20000;
const int resolution = 12;
const int dutyCycle = pow(2, resolution);*/

/*~~~~~~~~ Define Variables for RC receiver ~~~~~~~~*/
const int numChannels = sizeof receiveCh / sizeof * receiveCh;
volatile uint16_t interruptDuration;
volatile uint16_t channelDuration[] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint32_t lastInput;
uint16_t channelZero[] = {1500, 1500, 1500, 1500, 1500, 1500};
uint16_t channelMax[] = {2000, 2000, 2000, 2000, 2000, 2000};
uint16_t channelMin[] = {1000, 1000, 1000, 1000, 1000, 1000};

/*~~~~~~~~ Define Variables for rotary encoders ~~~~~~~~*/
const int numEncoderChannels = 4;
volatile uint16_t encoderCount[] = {0, 0, 0, 0};
volatile int8_t encoderDirection[] = {0, 0, 0, 0};
double wheelSpeed[] = {0, 0, 0, 0};
uint8_t invertEncoderDirection[] = {0, 0, 0, 0}; //for swapping polarity of encoder connections

/*~~~~~~~~ Define library object for TFT display ~~~~~~~~*/
//Adafruit_ILI9341 tft = Adafruit_ILI9341(screenPins[0], screenPins[1], screenPins[2], screenPins[3], screenPins[4], screenPins[5]);

/*~~~~~~~~ Define constants for inputs received via webserver ~~~~~~~~*/
const char* PARAM_INPUT_1 = "Max Throttle";
const char* PARAM_INPUT_2 = "Ramp Time";
const char* PARAM_INPUT_3 = "Kp";
const char* PARAM_INPUT_4 = "Ki";
const char* PARAM_INPUT_5 = "Kd";
const char* PARAM_INPUT_6 = "PID";

/*~~~~~~~~ Define Functions, Tasks, and ISRs ~~~~~~~~*/
void readPS4Input(float inputs[]);
void readRCInput(float inputs[]);
void calibrateRCInput(void);
void tftRewrite(char * oldString, char * newString, int x, int y, int textSize);
void motionControl();
void modbusMessage( char* address, char* function, char* data);
void stringtohex(char* input, char* output);
void rampInput(float inputs[]);
void TaskRunPlatform( void *pvParameters );
void TaskDisplay( void *pvParameters );
void TaskWirelessAccess( void *pvParameters );
void TaskSerialCommunication( void *pvParameters );
void TaskReadIMU( void *pvParameters );
void IRAM_ATTR pinChangeNoticeISR(void);
void IRAM_ATTR RotaryEncoderISR(void);

/*~~~~~~~~ FreeRTOS object declarations ~~~~~~~~*/
TaskHandle_t motionHandle;
QueueHandle_t displayQ;
SemaphoreHandle_t serialMutex;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; //for disabling ESP32 interrupts

/*~~~~~~~~ Define Variables for vehicle behavior adjustment ~~~~~~~~*/
volatile float rampTime = 0;
float speedMin = 10, speedMax = 100;
float rampMin = 0, rampMax = 40;

double maxSpeed = 5; //miles per hour, with gearing and 24v power supply ~7.7 mph max speed
double wheelDiameter = 1; //in feet
double gearRatio = 3*8.3;
double wheelCircumference = wheelDiameter * 3.14159265358979323846; // in feet
double maxRotationsPerSecond = maxSpeed*(5280./3600.) / wheelCircumference;

/*~~~~~~~~ PID Controller constants and setup~~~~~~~~*/
bool usePID = 0;
double scaledWheelSpeed[] = {0, 0, 0, 0}, PIDoutput[] = {0, 0, 0, 0}, targetSpeed[] = {0, 0, 0, 0};
double kp = .1, ki = .2, kd = 0;
double tuningMin = 0, tuningMax = 1;
double throttleMax = 1, throttleMin = -1 * throttleMax;
uint32_t sampleTime = 50; // milliseconds
PID PID1(&scaledWheelSpeed[0], &PIDoutput[0], &targetSpeed[0], kp, ki, kd, DIRECT);
PID PID2(&scaledWheelSpeed[1], &PIDoutput[1], &targetSpeed[1], kp, ki, kd, DIRECT);
PID PID3(&scaledWheelSpeed[2], &PIDoutput[2], &targetSpeed[2], kp, ki, kd, DIRECT);
PID PID4(&scaledWheelSpeed[3], &PIDoutput[3], &targetSpeed[3], kp, ki, kd, DIRECT);
PID *pidControllers[4] = {&PID1, &PID2, &PID3, &PID4};

/*~~~~~~~~ MPU9250 variables ~~~~~~~~*/
float heading = 0;
int useFixedCalibration = 0;
float fixedMagBias[] = {345.9, 179.81, -260.9};
float fixedMagScale[] = {1.11, .96, .95};

HardwareSerial IOExpander(1);

void setup() //Setup runs on Core 1
{
  Serial.begin(115200); //Enable serial communication for debugging
  Serial.println("Band on the Run: ESP32 Microcontroller");

  displayQ = xQueueCreate(1, sizeof(float) * 9);

  //serialMutex = xSemaphoreCreateBinary(); //
  //xSemaphoreGive(serialMutex);

  Serial.printf("Setup running on Core %d \n", xPortGetCoreID() );

  //xTaskCreate(TaskRunPlatform, "Platform Control", 4096, NULL, 1, &motionHandle);
  xTaskCreatePinnedToCore(TaskRunPlatform, "Platform Control", 4096, NULL, 1, &motionHandle, 0); //Must be on different core than TaskReadIMU
  //xTaskCreate(TaskDisplay,  "OLED",  4096 ,  NULL,  1,  NULL );
  xTaskCreatePinnedToCore(TaskReadIMU,  "IMU",  4096 ,  NULL,  1,  NULL, 1 );
  //xTaskCreate(TaskReadIMU,  "IMU",  4096 ,  NULL,  1,  NULL );
  xTaskCreatePinnedToCore(TaskWirelessAccess, "WiFi Service", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  //empty
}

void TaskRunPlatform( void *pvParameters )
{
  TickType_t startTick;
  static TickType_t printTick;
  uint32_t startTime, lastTime;

  Serial.printf("Run Platform task started on Core %d. \n", xPortGetCoreID() );
  
  /*** Setup controller input ***/
  for (int i = 0; i < numChannels; i++) //~~~RC Controller~~~//
  {
    pinMode(receiveCh[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(receiveCh[i]), pinChangeNoticeISR, CHANGE);
  }

  /***~~~ Setup outputs ~~~***/
  //while( !xSemaphoreTake(serialMutex, 1) ){}
  IOExpander.begin(400000, SERIAL_8N1, IOExpanderPins[0], IOExpanderPins[1]);
  //xSemaphoreGive(serialMutex);

  /***~~~ Setup rotary encoders ~~~***/
  for (int i = 0; i < 4; i++)
  {
    pinMode(encoderCh[i * 2], INPUT);
    pinMode(encoderCh[i * 2 + 1], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderCh[i * 2]), RotaryEncoderISR, CHANGE);
  }
  /***~~~ Setup PID controllers ~~~***/
  for (int i = 0; i < 4; i++)
  {
    pidControllers[i]->SetOutputLimits(throttleMin, throttleMax);
    pidControllers[i]->SetSampleTime(sampleTime);
  }

  for (;;)
  {
    startTime = micros();
    startTick = xTaskGetTickCount();

    for (int i = 0; i < 4; i++)
    {
      wheelSpeed[i] = encoderCount[i] / 100. * (1000000. / (startTime - lastTime )) * encoderDirection[i] / gearRatio;
      scaledWheelSpeed[i] = wheelSpeed[i] / maxRotationsPerSecond;
      
      encoderCount[i] = 0;
    }

    motionControl();

    #ifdef printDebugStatements
    if(xTaskGetTickCount() - printTick > printInterval) {
      printTick = xTaskGetTickCount();
      Serial.printf("Time for control loop: %d microseconds \n", (micros() - startTime));
      for(int i = 0; i < 4; i++)
        Serial.printf("Encoder %d rotations per second: %f direction: %d \n", i, wheelSpeed[i], encoderDirection[i]);
      
    }
    #endif

    lastTime = startTime;
    vTaskDelayUntil( &startTick, sampleTime );
  }
}

void TaskReadIMU( void *pvParameters )
{
  float headings[3] = {};
  static TickType_t printTick, startTick;
  uint32_t startTime;

  startTick = xTaskGetTickCount();
  
  Serial.printf("Read IMU task started on Core %d.\n", xPortGetCoreID() );

  Wire.begin(I2CPins[0], I2CPins[1], 400000);

  for(;;)
  {
    if(initHeadingReading(fixedMagBias, fixedMagScale, useFixedCalibration))
      break;

    #ifdef printDebugStatements
    Serial.printf("Could not connect to Inertial Motion Unit.\n");
    #endif
    
    vTaskDelay(printInterval);
  }

  for(int i = 0; ; i++)
  {
    startTick = xTaskGetTickCount();
    startTime = micros();
    
    //while( !xSemaphoreTake(serialMutex, 1) ){}
    if( !updateHeading(headings) ) //if connection lost
    {
      for(;;)
      {
        if(initHeadingReading(fixedMagBias, fixedMagScale, useFixedCalibration))
          break;
    
        #ifdef printDebugStatements
        Serial.printf("Could not connect to Inertial Motion Unit.\n");
        #endif
        
        vTaskDelay(printInterval);
      }
    }
    //xSemaphoreGive(serialMutex);
    
    heading = headings[0];
    
    #ifdef printDebugStatements
    if(startTick - printTick > printInterval) {
      printTick = xTaskGetTickCount();
      Serial.printf("Yaw: %.2f  Pitch: %.2f  Roll: %.2f \n", headings[0], headings[1], headings[2]);
      Serial.printf("Time for heading calculation loop: %d microseconds \n", (micros() - startTime));
    }
    #endif
    
    vTaskDelayUntil( &startTick, 100 );
  }
}

void TaskWirelessAccess( void *pvParameters )
{
  DNSServer dnsServer;
  AsyncWebServer server(80);

  Serial.printf("Wireless access task started on Core %d\n", xPortGetCoreID() );

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  Serial.println(WiFi.softAPIP());

  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
    String inputMessage;
    String inputParam;
    bool goodMessage = false;

    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      if (inputMessage.toFloat() >= speedMin && inputMessage.toFloat() <= speedMax)
      {
        throttleMax = inputMessage.toFloat() / 100;
        throttleMin = throttleMax * -1;
        for (int i = 0; i < 4; i++)
          pidControllers[i]->SetOutputLimits(throttleMin, throttleMax);
        goodMessage = true;
      }
    }
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
      if (inputMessage.toFloat() >= rampMin && inputMessage.toFloat() <= rampMax)
      {
        rampTime = inputMessage.toFloat();
        goodMessage = true;
      }
    }
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3;
      if (inputMessage.toFloat() >= tuningMin && inputMessage.toFloat() <= tuningMax)
      {
        kp = inputMessage.toFloat();
        for (int i = 0; i < 4; i++)
          pidControllers[i]->SetTunings(kp, ki, kd);
        goodMessage = true;
      }
    }
    else if (request->hasParam(PARAM_INPUT_4)) {
      inputMessage = request->getParam(PARAM_INPUT_4)->value();
      inputParam = PARAM_INPUT_4;
      if (inputMessage.toFloat() >= tuningMin && inputMessage.toFloat() <= tuningMax)
      {
        ki = inputMessage.toFloat();
        for (int i = 0; i < 4; i++)
          pidControllers[i]->SetTunings(kp, ki, kd);
        goodMessage = true;
      }
    }
    else if (request->hasParam(PARAM_INPUT_5)) {
      inputMessage = request->getParam(PARAM_INPUT_5)->value();
      inputParam = PARAM_INPUT_5;
      if (inputMessage.toFloat() >= tuningMin && inputMessage.toFloat() <= tuningMax)
      {
        kd = inputMessage.toFloat();
        for (int i = 0; i < 4; i++)
          pidControllers[i]->SetTunings(kp, ki, kd);
        goodMessage = true;
      }
    }

    else if (request->hasParam(PARAM_INPUT_6)) {
      inputMessage = request->getParam(PARAM_INPUT_6)->value();
      inputParam = PARAM_INPUT_6;
      if (inputMessage.toFloat() == 0 || inputMessage.toFloat() == 1)
      {
        usePID = inputMessage.toInt();
        //for (int i = 0; i < 4; i++)
          //pidControllers[i]->SetTunings(kp, ki, kd);
        goodMessage = true;
      }
    }

    if (!goodMessage) {
      inputMessage = "Invalid Value";
      //inputParam = "none";
    }
    Serial.println(inputMessage);
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field ("
                  + inputParam + ") with value: " + inputMessage +
                  "<br><a href=\"/\">Return to Home Page</a>");
  });

  server.on("", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->printf("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>\n");
    response->printf("<h1>Band on the Run</h1>\n");
    response->printf("<p>Maximum throttle (pwm duty cycle): %.0f%% \n", throttleMax * 100);
    response->printf("<p>Ramp Time: %.2f seconds\n", rampTime);
    response->printf("<p>Use PID: %d \n", usePID);
    response->printf("<p>Kp: %.4f \n", kp);
    response->printf("<p>Ki: %.4f \n", ki);
    response->printf("<p>Kd: %.4f \n", kd);

    response->printf("<form action=\"/get\">\n");
    response->printf("New max throttle (%.0f - %.0f): <input type=\"text\" name=\"Max Throttle\">\n", speedMin, speedMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");

    response->printf("<form action=\"/get\">\n");
    response->printf("New Ramp Time (%.0f - %.0f): <input type=\"text\" name=\"Ramp Time\">\n", rampMin, rampMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");

    response->printf("<form action=\"/get\">\n");
    response->printf("Use PID (0 or 1): <input type=\"text\" name=\"PID\">\n");
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");

    response->printf("<form action=\"/get\">\n");
    response->printf("New Kp (%f - %f): <input type=\"text\" name=\"Kp\">\n", tuningMin, tuningMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");

    response->printf("<form action=\"/get\">\n");
    response->printf("New Ki (%f - %f): <input type=\"text\" name=\"Ki\">\n", tuningMin, tuningMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");

    response->printf("<form action=\"/get\">\n");
    response->printf("New Kd (%f - %f): <input type=\"text\" name=\"Kd\">\n", tuningMin, tuningMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");


    response->print("</body></html>");
    request->send(response);
  });

  server.begin();
  for (;;)
  {
    dnsServer.processNextRequest();
    vTaskDelay(100);
  }
}

void motionControl()
{
  static float inputs[5] = {0, 0, 0, 0, 0};
  static float wheelOutput[4] = {0, 0, 0, 0};
  static float wheelUse[4] = {0, 0, 0, 0};
  static float sendData[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  char outputMessage[256] = {};
  const char IOExpanderKey[] = "85NtAb5Dv8f7pcmb";
  static TickType_t printTick, startTick;

  startTick = xTaskGetTickCount();

  /***~~~ Check if controller connected, read input from controller ~~~***/
  if (controllerSelected == Playstation)
  {
    //readPS4Input(inputs);
  }
  else if (controllerSelected == RC)
  {
    readRCInput(inputs);
  }

  /***** FailSafe to stop Platform *******/
  if ( inputs[timeout] )
  {
    for (int i = 0; i < 4; i++)
      inputs[i] = 0;

#ifdef printDebugStatements
    if(startTick - printTick > printInterval) 
    {
      Serial.println("No controller connected!");
    }
#endif
  }
  /********** End FailSafe *************/

  #ifdef printDebugStatements
    if(startTick - printTick > printInterval) 
    {
      for(int i = 0; i < 4; i++)
        Serial.printf("Input %d: %f \n", i, inputs[i]);
    }
#endif

  /*~~~ Determine wheel speed from inputs ~~~*/
  if (controllerSelected == RC)
  {
    wheelUse[Front] = inputs[lxStick] + inputs[rxStick];
    wheelUse[Right] = -inputs[lyStick] + inputs[rxStick];
    wheelUse[Back] = -inputs[lxStick] + inputs[rxStick];
    wheelUse[Left] = inputs[lyStick] + inputs[rxStick];
    //wheelUse[Front] = inputs[lyStick]; //single wheel from left y for testing

  }

  /*if (controllerSelected == Playstation)
  {
    wheelUse[Front] = inputs[lxStick] + inputs[rxStick];
    wheelUse[Right] = -inputs[lyStick] + inputs[rxStick];
    wheelUse[Back] = -inputs[lxStick] + inputs[rxStick];
    wheelUse[Left] = inputs[lyStick] + inputs[rxStick];
  }*/

#ifdef printDebugStatements
  if(startTick - printTick > printInterval) 
  {
    Serial.printf("Wheels from input: %f  %f  %f  %f\n", wheelUse[0], wheelUse[1], wheelUse[2], wheelUse[3]);
  }
#endif

  /*~~~ Create deadzone for input so platform doesnt move unexpectedly ~~~*/
  if ( (abs(wheelUse[Front]) < .1) &&
       (abs(wheelUse[Right]) < .1) &&
       (abs(wheelUse[Back]) < .1) &&
       (abs(wheelUse[Left]) < .1) )
  {
#ifdef printDebugStatements
  if(startTick - printTick > printInterval) 
  {
    Serial.printf("Controller deadzone\n");
  }
#endif

    for (int i = 0; i < 4; i++)
    {
      wheelUse[i] = 0;
    }

    /*~~~ If controller in deadzone and platform stopped, disable PID control and force output to zero ~~~*/
    if ( (usePID == 1) &&
         (abs(wheelSpeed[Front]) < .1) &&
         (abs(wheelSpeed[Right]) < .1) &&
         (abs(wheelSpeed[Back]) < .1) &&
         (abs(wheelSpeed[Left]) < .1) )
    {
      for (int i = 0; i < 4; i++)
      {
        pidControllers[i]->SetMode(MANUAL);
        PIDoutput[i] = 0;
      }
    }
  }
  else
  {
    if ( usePID == 1 )
    {
      for (int i = 0; i < 4; i++)
      {
        pidControllers[i]->SetMode(AUTOMATIC);
      }
    }
  }

  /*~~~ Scale wheel speeds down proportionally in case of both translation and rotation~~~*/
  for (int i = 0; i < 4; i++)
  {
    if ( abs(wheelUse[i]) > 1 )
    {
#ifdef printDebugStatements
      if(startTick - printTick > printInterval) 
      {
        Serial.printf("Wheel %i exceeds maximum\n", i);
      }
#endif

      for (int j = i + 1; j != i; j = (j + 1) % 4)
      {
        wheelUse[j] = wheelUse[j] / abs(wheelUse[i]);
      }
      wheelUse[i] = wheelUse[i] / abs(wheelUse[i]);
    }
  }

  if (usePID == 1)
  {
    for (int i = 0; i < 4; i++)
    {
      uint32_t startLoop = micros();
      
      targetSpeed[i] = wheelUse[i];
      pidControllers[i]->Compute();
      wheelOutput[i] = PIDoutput[i];
    }
    
#ifdef printDebugStatements
    if(startTick - printTick > printInterval) 
    {
      for (int k = 0; k < 4; k++)
        Serial.printf("PID %d Input: %.2f (-1 to 1) Target: %.2f (-1 to 1) Output: %.2f (%.2f - %.2f)\n", k, scaledWheelSpeed[k], targetSpeed[k], PIDoutput[k], throttleMin, throttleMax);
    }
#endif
  
  }
  else
  {
    for (int i = 0; i < 4; i++)
      wheelUse[i] = wheelUse[i] * throttleMax;
    rampInput(wheelUse);
    for (int i = 0; i < 4; i++)
    {
      wheelOutput[i] = wheelUse[i];
    }
  }

  /*~~~ Apply output signals ~~~*/
  sprintf(outputMessage, "Wheel1: %f \nWheel2: %f \nWheel3: %f \nWheel4: %f \nIOExpanderKey: %s\n", wheelOutput[0], wheelOutput[1], wheelOutput[2], wheelOutput[3], IOExpanderKey);
  //while( !xSemaphoreTake(serialMutex, 1) ){}
  IOExpander.write((uint8_t *) outputMessage, sizeof(outputMessage));
  IOExpander.flush();
  //xSemaphoreGive(serialMutex);
    
#ifdef printDebugStatements
  if(startTick - printTick > printInterval) 
  {
    Serial.printf("IO Expander Message: \n%s", outputMessage);
    printTick = xTaskGetTickCount();
  }
#endif

  memcpy(&sendData[0], inputs, sizeof(inputs));
  memcpy(&sendData[5], wheelOutput, sizeof(wheelOutput));

  xQueueOverwrite(displayQ, &sendData);
}


void readRCInput(float inputs[])
{
  long localChannelDuration[numChannels];
  uint32_t localLastInput;
  float range[numChannels];
  float inputValue[numChannels];
  static TickType_t printTick;

  for (int i = 0; i < numChannels; i++)
    localChannelDuration[i] = channelDuration[i];

  localLastInput = lastInput;

  /*~~~~~~ Determine Stick Positions channels 1 - 3 ~~~~~~*/
  for (int i = 0; i < 4; i++)
  {
    if ( (channelMax[i] != 0) && (channelZero[i] != 0) && (channelMax[i] != channelZero[i]))
    {
      if (localChannelDuration[i] >= channelZero[i])
      {
        inputValue[i] = (float) (localChannelDuration[i] - channelZero[i]) / (float) (channelMax[i] - channelZero[i]);
      }
      else
      {
        inputValue[i] = -((float) (channelZero[i] - localChannelDuration[i] ) / (float) ( channelZero[i] - channelMin[i]));
      }
    }
  }

  for (int i = 0; i < 4; i++)
    if ( inputValue[i] > 1 )
      inputValue[i] = 1;
    else if ( inputValue[i] < -1 )
      inputValue[i] = -1;

  inputs[rxStick] = inputValue[0];
  inputs[ryStick] = inputValue[1];
  inputs[lyStick] = inputValue[2];
  inputs[lxStick] = inputValue[3];

  /*~~~~~~ Determine switch status channel 4 ~~~~~~*/
  if (localChannelDuration[4] > 1800)
  {
    calibrateRCInput();
  }

  /*~~~ Check for controller timeout ~~~*/
  uint32_t timeSinceInput = (micros() - localLastInput);
  if ( timeSinceInput > 25000 )
  {
#ifdef printDebugStatements
  if(xTaskGetTickCount() - printTick > printInterval) {
    //printTick = xTaskGetTickCount();
    Serial.println("Controller timeout");
  }
#endif

    inputs[timeout] = 1;
  }
  else
  {
#ifdef printDebugStatements
    if(xTaskGetTickCount() - printTick > printInterval) {
      Serial.println("Controller CONNECTED");
    }
#endif

    inputs[timeout] = 0;
  }

#ifdef printDebugStatements
  if(xTaskGetTickCount() - printTick > printInterval) {
    printTick = xTaskGetTickCount();
    Serial.println("Channel Status");
    for (int i = 0; i < numChannels; i++)
    {
      Serial.printf("Min: %d   Zero: %d   Max: %d  Current: %d \n",
                    channelMin[i], channelZero[i], channelMax[i], localChannelDuration[i]);
    }
  }
#endif
}

void calibrateRCInput(void)
{
  long localChannelDuration[numChannels];
  int set = 1;

#ifdef printDebugStatements
  Serial.println("Entering RC Calibration");
#endif

  do
  {
    for (int i = 0; i < numChannels; i++)
      localChannelDuration[i] = channelDuration[i];

    //Set min and max to current value on entering calibration
    if (set == 1)
    {
      for (int i = 0; i < numChannels - 1; i++)
      {
        channelMax[i] = localChannelDuration[i];
        channelMin[i] = localChannelDuration[i];
      }
      set = 0;
    }

    //Read inputs and set minimum and maximum pulse duration
    else
    {
      for (int i = 0; i < numChannels; i++)
      {
        if (localChannelDuration[i] > channelMax[i])
          channelMax[i] = localChannelDuration[i];
        else if (localChannelDuration[i] < channelMin[i])
          channelMin[i] = localChannelDuration[i];
      }
    }
    delay(50);

#ifdef printDebugStatements
    Serial.println("Channel Status");
    for (int i = 0; i < numChannels; i++)
    {
      Serial.printf("Min: %d   Zero: %d   Max: %d  Current: %d \n",
                    channelMin[i], channelZero[i], channelMax[i], localChannelDuration[i]);
    }
#endif

    for (int i = 0; i < numChannels - 1; i++)
      channelZero[i] = ( (channelMax[i] - channelMin[i]) / 2 ) + channelMin[i];

  } while ( localChannelDuration[4] > 1750 );

#ifdef printDebugStatements
  Serial.println("Leaving RC Calibration");
#endif
}


/*** Limits change in throttle determined by the ramp time value (ramp time = time for throttle to go from 0% to 100%) ***/
void rampInput(float inputs[])
{
  static uint32_t oldTime = 0, newTime = 0, timeDiff = 0;
  static float currentSpeed[4] = {0, 0, 0, 0};
  float speedDiff;

  if (rampTime == 0) //handle divide by 0, also case where function has no effect
    return;
  else
    speedDiff = 1 / rampTime / 1000000;

  //static float dt = 1 / rampTime / 1000000;
  newTime = micros();

  if (oldTime == 0)
    oldTime = newTime; // prevent jump from first call

  timeDiff = newTime - oldTime;
#ifdef printDebugStatements
    static TickType_t printTick;
    if(xTaskGetTickCount() - printTick > printInterval) 
    {
      printTick = xTaskGetTickCount();
      Serial.printf("Ramp input values -- \nTime Diff: %d ms \ndt: %f \ncurrentSpeeds: %f %f %f %f\n", timeDiff / 1000, speedDiff * 1000, currentSpeed[0], currentSpeed[1], currentSpeed[2], currentSpeed[3]);
    }
#endif

  for (int i = 0; i < 4; i++)
  {
    if ( inputs[i] > currentSpeed[i] + speedDiff * timeDiff )
      inputs[i] = currentSpeed[i] + speedDiff * timeDiff;
    else if ( inputs[i] < currentSpeed[i] - speedDiff * timeDiff )
      inputs[i] = currentSpeed[i] - speedDiff * timeDiff;
    else
      inputs[i] = inputs[i];
    currentSpeed[i] = inputs[i];
  }
  oldTime = newTime;
}

/*void TaskDisplay( void *pvParameters )
{
  float localInputs[9] = {}, lastInputs[9] = {}, lastText[16] = {};
  const uint16_t dotSize = 12, halfDot = dotSize / 2,
                 xRect1 = 30, xRect2 = 190, yRect = 50,
                 boxLength = 100, halfLength = boxLength / 2;
  uint32_t stickTimer = 0, textTimer = 0;
  char newText[10] = {}, oldText[10] = {};

  lastText[timeout] = -1; //ensure initial drawing of controller status
  lastText[5] = -1.11; //ensure initial drawing of wheel speed
  lastText[10] = -1.11; //ensure initial drawing of ramp time
  lastText[12] = -1.11;
  lastInputs[0] = -.555;  //ensure initial draw of stick dots
  lastInputs[2] = -.555;  //ensure initial draw of stick dots


  Serial.println("TFT Display task started.");

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setRotation(3);
  tft.setTextSize(2);

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(60, 0);
  tft.printf("Band on the Run\n");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 170);
  tft.printf("Throttle:");
  tft.setCursor(0, 193);
  tft.printf("Accel:");
  tft.setCursor(0, 216);
  tft.printf("Ramp Time:");

  tft.setCursor(180, 216);
  tft.printf("Yaw:");

  tft.drawRect(xRect1, yRect, boxLength + dotSize + 2, boxLength + dotSize + 2, ILI9341_WHITE);
  tft.drawRect(xRect2, yRect, boxLength + dotSize + 2, boxLength + dotSize + 2, ILI9341_WHITE);

  tft.setRotation(2);
  tft.setTextSize(1);
  tft.setCursor(105, 20);
  tft.printf("Left Stick");

  tft.setCursor(105, 180);
  tft.printf("Right Stick");

  tft.setRotation(3);
  tft.setTextSize(2);

  for (;;)
  {
    xQueuePeek(displayQ, &localInputs, 0);

    if (millis() > textTimer + 300)
    {
      textTimer = millis();
      if ( (localInputs[timeout] == 1) && (localInputs[timeout] != lastText[timeout]) )
      {
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(0, 25);
        tft.printf("No controller connected");
        tft.setCursor(0, 25);
        tft.printf("Controller connection good");
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(0, 25);
        tft.printf("No controller connected");
      }
      else if ( (localInputs[timeout] == 0) && (localInputs[timeout] != lastText[timeout]) )
      {
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(0, 25);
        tft.printf("No controller connected");
        tft.setCursor(0, 25);
        tft.printf("Controller connection good");
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(0, 25);
        tft.printf("Controller connection good");
      }

      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%0.2f", localInputs[5]);
      sprintf(oldText, "%0.2f", lastText[5]);
      tftRewrite(oldText, newText, 125, 170, 2);

      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%.0f%%", throttleMax * 100);
      sprintf(oldText, "%.0f%%", lastText[9] * 100);
      tftRewrite(oldText, newText, 130, 193, 2);

      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%.1f", rampTime);
      sprintf(oldText, "%.1f", lastText[10]);
      tftRewrite(oldText, newText, 125, 216, 2);

      float _heading = heading;
      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      
      sprintf(newText, "%.2f", _heading);
      sprintf(oldText, "%.2f", lastText[11]);
      tftRewrite(oldText, newText, 228, 216, 2);

      float _wheelSpeed[4];

      for(int i = 0; i < 4; i ++)
        _wheelSpeed[i] = wheelSpeed[i];

      
      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%.2f", _wheelSpeed[0]);
      sprintf(oldText, "%.2f", lastText[12]);
      tftRewrite(oldText, newText, 200, 170, 2);

      memcpy(lastText,  localInputs, sizeof(localInputs));
      lastText[9] = throttleMax;
      lastText[10] = rampTime;
      lastText[11] = _heading;
      
      memcpy(&lastText[12], _wheelSpeed, sizeof(_wheelSpeed));
      //lastText[12] = _wheelSpeed[0];
    }

    if (millis() > stickTimer + 100)
    {
      stickTimer = millis();
      if ( (lastInputs[lxStick] != localInputs[lxStick]) || (lastInputs[lyStick] != localInputs[lyStick]) )
      {
        tft.fillRect(xRect1 + halfLength + 1 + (halfLength * lastInputs[lxStick]), yRect + halfLength + 1 - (halfLength * lastInputs[lyStick]), dotSize, dotSize, ILI9341_BLACK);
        tft.fillRect(xRect1 + halfLength + 1 + (halfLength * localInputs[lxStick]), yRect + halfLength + 1 - (halfLength * localInputs[lyStick]), dotSize, dotSize, ILI9341_BLUE);
      }

      if ( (lastInputs[rxStick] != localInputs[rxStick]) || (lastInputs[ryStick] != localInputs[ryStick]) )
      {
        tft.fillRect(xRect2 + halfLength + 1 + (halfLength * lastInputs[rxStick]), yRect + halfLength + 1 - (halfLength * lastInputs[ryStick]), dotSize, dotSize, ILI9341_BLACK);
        tft.fillRect(xRect2 + halfLength + 1 + (halfLength * localInputs[rxStick]), yRect + halfLength + 1 - (halfLength * localInputs[ryStick]), dotSize, dotSize, ILI9341_BLUE);
      }
      memcpy(lastInputs,  localInputs, sizeof(localInputs));
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}*/


/*** Redraws the characters on the display one at a time to reduce flickering ***/
/*void tftRewrite(char * oldString, char * newString, int x, int y, int textSize)
{
  tft.setTextSize(textSize);
  int textLength = 0;

  if (strlen(oldString) >= strlen(newString))
    textLength = strlen(oldString);
  else
    textLength = strlen(newString);

  for (int i = 0; i < textLength; i++)
  {
    if (oldString[i] != newString[i])
    {
      if (oldString[i] != NULL)
      {
        tft.setCursor(x + i * textSize * 6, y);
        tft.setTextColor(ILI9341_BLACK);
        tft.printf("%c", oldString[i]);
      }

      if (newString[i] != NULL)
      {
        tft.setCursor(x + i * textSize * 6, y);
        tft.setTextColor(ILI9341_WHITE);
        tft.printf("%c", newString[i]);
      }
    }
  }
}*/

/*** ISR for reading the pulse widths from Flysky RC Receiver ***/
void IRAM_ATTR pinChangeNoticeISR(void)
{
  portENTER_CRITICAL_ISR(&mux); //Disable interrupts within ISR

  static uint16_t startTimer, channelTimer[numChannels];
  static uint8_t channelVal[numChannels], channelOld[numChannels];

  startTimer = micros();

  //Save last values
  for (int i = 0; i < numChannels; i++)
    channelOld[i] = channelVal[i];

  //Read inputs
  for (int i = 0; i < numChannels; i++)
    channelVal[i] = digitalRead(receiveCh[i]);

  //If transition occured, either start timer or set pulse duration
  for (int i = 0; i < numChannels; i++)
  {
    if (channelVal[i] > channelOld[i])
    {
      channelTimer[i] = micros();
    }
    else if (channelVal[i] < channelOld[i])
    {
      channelDuration[i] = micros() - channelTimer[i];
      lastInput = micros();
    }
  }
  interruptDuration = micros() - startTimer;
  portEXIT_CRITICAL_ISR(&mux); //Re-enable interrupts
}

/*** ISR for counting pulses and determining direction from rotary encoders ***/
void IRAM_ATTR RotaryEncoderISR(void)
{
  static uint8_t valueRead[4], directionRead[4], valueOld[4];

  for (int i = 0; i < numEncoderChannels; i++)
  {
    valueOld[i] = valueRead[i];
    valueRead[i] = digitalRead(encoderCh[i * 2]);

    if (valueRead[i] > valueOld[i])
    {
      encoderDirection[i] = ((digitalRead(encoderCh[i * 2 + 1]) * 2) - 1) * (invertEncoderDirection[i] ? -1 : 1); //sets direction as -1 or 1
      encoderCount[i]++;
      //Serial.printf("Encoder %d pulse! Direction %d\n", i, encoderDirection[i]);
    }
  }
}
