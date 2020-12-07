/*************************************************
   Band on the Run
   12 October 2020

   miniMe
   Version 2.1

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

   -Adruino IDE seems to have some issue with ESP32 bluetooth,
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
   Split RS485 communication into separate task. Limited scope of objects to their task.
   PS4 Controller support commented out due to exceeding maximum program memory.
   Adjustment of speed limit and ramp time through captive portal added.


   ~~MiniMe~~
   2.0 
   Converted new fullscale design to the work on the miniMe.

   2.1 
   Added wheel speed measurement code and PID control.

*/

#include <PS4Controller.h>
//#include <WiFi.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <DNSServer.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_ILI9341.h>
#include <PID_v1.h>


#define PS4ControllerMAC "00:15:83:F2:82:BF"
#define printDebugStatements //Leave defined for serial debugging messages

/*~~~ Access point setup ~~~*/
const char* ssid     = "Band on the Run";
const char* password = "thisisatest";
const byte DNS_PORT = 53;

/*~~~ Choose Controller ~~~*/
enum controller {Playstation, RC};
int controllerSelected = Playstation;

/*********Pin Assignments*********/
const int forwardPin[] = {0, 27, 0, 0}; //Define pins for motor direction control
const int backwardPin[] = {0, 26, 0, 0};  //Define pins for motor speed control
const int receiveCh[] = {0,0,0,0,0}; //pin assignment for the RC Receiver
const int screenPins[] = {0,0,0,0,0,0}; //pin assignment for TFT display
const int encoderCh[] = {0, 0, 25, 33, 0, 0, 0, 0}; //Pins for rotary encoder

/*const int directionPin[] = {18, 0, 0, 0}; //Define pins for motor direction control
const int speedPin[] = {19, 0, 0, 0};  //Define pins for motor speed control
const int receiveCh[] = {32, 35, 34, 39, 36}; //pin assignment for the RC Receiver
const int screenPins[] = {12, 27, 26, 25, 14, 0}; //pin assignment for TFT display
const int encoderCh[] = {0, 0, 0, 0}; //Pins for rotary encoder*/
/*********End Pin Assignments*********/

/*~~~~~~~~ Define enums to make wheel and input array values easier to read ~~~~~~~~*/
enum wheel {Front, Right, Back, Left};
enum input {lxStick, lyStick, rxStick, ryStick, timeout};

/*~~~~~~~~ Define Variables for PWM channels ~~~~~~~~*/
const int pwmCh[] = {0, 1, 2, 3};     //Define channels for internal ESP32 PWM generator
const int freq = 20000;
const int resolution = 12;
const int dutyCycle = pow(2, resolution);

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
double wheelSpeed[] = {0,0,0,0};
uint8_t invertEncoderDirection[] = {0,1,0,0};

/*~~~~~~~~ Define library object for TFT display ~~~~~~~~*/
//Adafruit_ILI9341 tft = Adafruit_ILI9341(screenPins[0], screenPins[1], screenPins[2], screenPins[3], screenPins[4], screenPins[5]);

/*~~~~~~~~ Define constants for inputs received via webserver ~~~~~~~~*/
const char* PARAM_INPUT_1 = "Speed Limiter";
const char* PARAM_INPUT_2 = "Ramp Time";

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
void IRAM_ATTR pinChangeNoticeISR(void);
void IRAM_ATTR RotaryEncoderISR(void);

/*~~~~~~~~ FreeRTOS object declarations ~~~~~~~~*/
TaskHandle_t motionHandle;
QueueHandle_t displayQ;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; //for disabling ESP32 interrupts

/*~~~~~~~~ Define Variables for vehicle behavior adjustment ~~~~~~~~*/
volatile float speedLimiter = 1;
volatile float rampTime = 2;
float speedMin = 10, speedMax = 100;
float rampMin = 0, rampMax = 40;

double maxSpeed = 5; //feet per second
double wheelDiameter = 1; 
double wheelCircumference = wheelDiameter * 3.14159265358979323846;
double maxRotationsPerSecond = maxSpeed / wheelCircumference;

/*~~~~~~~~ PID Controller Setup~~~~~~~~*/
bool usePID = 1;
double scaledWheelSpeed[] = {0,0,0,0}, output[] = {0,0,0,0}, targetSpeed[] = {0,0,0,0};
double kp = .04, ki = .1, kd = 0;
double throttleMax = .5, throttleMin = -1*throttleMax;
uint32_t sampleTime = 100;
PID PID1(&scaledWheelSpeed[0], &output[0], &targetSpeed[0], kp, ki, kd, DIRECT);
PID PID2(&scaledWheelSpeed[1], &output[1], &targetSpeed[1], kp, ki, kd, DIRECT);
PID PID3(&scaledWheelSpeed[2], &output[2], &targetSpeed[2], kp, ki, kd, DIRECT);
PID PID4(&scaledWheelSpeed[3], &output[3], &targetSpeed[3], kp, ki, kd, DIRECT);
PID *pidControllers[4] = {&PID1, &PID2, &PID3, &PID4};

void setup()
{
  Serial.begin(115200); //Enable serial communication for debugging
  Serial.println("Band on the Run: ESP32 Microcontroller");

  pinMode(2, OUTPUT); // led pin

  displayQ = xQueueCreate(1, sizeof(float) * 9);

  xTaskCreate(TaskRunPlatform, "Platform Control", 4096, NULL, 1, &motionHandle);
  //xTaskCreate(TaskSerialCommunication, "Serial Communication", 4096, NULL, 1, NULL);
  //xTaskCreate(TaskDisplay,  "OLED",  4096 ,  NULL,  1,  NULL );
  //xTaskCreatePinnedToCore(TaskWirelessAccess, "WiFi Service", 4096, NULL, 1, NULL, 0);
}

void loop()
{
  //empty
}



/*void TaskWirelessAccess( void *pvParameters )
{
  DNSServer dnsServer;
  AsyncWebServer server(80);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  Serial.println("Wireless access task started.");
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
        speedLimiter = inputMessage.toFloat() / 100;
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
    response->printf("<p>Speed Limiter: %.0f%% \n", speedLimiter * 100);
    response->printf("<p>Ramp Time: %.2f seconds\n", rampTime);
    response->printf("<form action=\"/get\">\n");
    response->printf("New Speed Limiter (%.0f - %.0f): <input type=\"text\" name=\"Speed Limiter\">\n", speedMin, speedMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");
    response->printf("<form action=\"/get\">\n");
    response->printf("New Ramp Time (%.0f - %.0f): <input type=\"text\" name=\"Ramp Time\">\n", rampMin, rampMax);
    response->printf("<input type=\"submit\" value=\"Submit\">\n");
    response->printf("</form><br>\n");
    response->print("</body></html>");
    request->send(response);
  });

  server.begin();
  for (;;)
  {
    dnsServer.processNextRequest();
    vTaskDelay(10);
  }
}*/

void TaskRunPlatform( void *pvParameters )
{
  TickType_t startTime, lastTime;

  /*** Setup controller input ***/
  PS4.begin(PS4ControllerMAC); //~~~PS4 Controller~~~//
  
  for (int i = 0; i < numChannels; i++) //~~~RC Controller~~~//
  {
    pinMode(receiveCh[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(receiveCh[i]), pinChangeNoticeISR, CHANGE);
  }

  /***~~~ Setup outputs ~~~***/
  for (int i = 0; i < 4; i++)
  {
    //pinMode(directionPin[i], OUTPUT);
    ledcSetup(pwmCh[i], freq, resolution);
    ledcWrite(pwmCh[i], 0); //initialize duty cycle to 0
    //ledcAttachPin(speedPin[i], pwmCh[i]);
  }

  /***~~~ Setup rotary encoders ~~~***/
  for (int i = 0; i < 4; i++)
  {
    pinMode(encoderCh[i*2], INPUT);
    pinMode(encoderCh[i*2+1], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderCh[i*2]), RotaryEncoderISR, CHANGE);
  }
  /***~~~ Setup PID controllers ~~~***/
  for (int i = 0; i < 4; i++)
  {
    pidControllers[i]->SetOutputLimits(throttleMin, throttleMax);
    pidControllers[i]->SetSampleTime(sampleTime);
  }
  
  Serial.println("Run Platform task started.");

  for (;;)
  {
    startTime = xTaskGetTickCount();

    #ifdef printDebugStatements
      Serial.println("Running platform control");
    #endif

    for(int i = 0; i < 4; i++)
    {
      wheelSpeed[i] =   encoderCount[i] / 100. * (1000. / (startTime - lastTime )) * encoderDirection[i];
      scaledWheelSpeed[i] = wheelSpeed[i] / maxRotationsPerSecond;
      Serial.printf("Encoder %d rotations per second: %f direction: %d \n", i, wheelSpeed[i], encoderDirection[i]);
      encoderCount[i] = 0;
      //wheelSpeed[i] = 0;
    }

    motionControl(); 
    
    //printf("revPerSec %f direction %s \n", wheelSpeed[0], encoderDirection[0] ? "forward" : "backward");
    Serial.printf("Time for control loop: %d ms \n", (startTime - lastTime));
    lastTime = startTime;
    vTaskDelayUntil( &startTime, sampleTime );
  }
}

void TaskSerialCommunication( void *pvParameters )
{
  TickType_t startTime;

  HardwareSerial RS485(1);
  RS485.begin(9600, SERIAL_8N1, 22, 23);

  Serial.println("RS485 Serial communication task started.");

  for (;;)
  {
    uint8_t buffer[200] = {};
    startTime = xTaskGetTickCount();

    //modbusMessage("test");
    vTaskDelayUntil( &startTime, 100 );

    if (RS485.available() > 0)
    {
      uint16_t i = 0;
      while (RS485.available() > 0)
      {
        buffer[i] = RS485.read();
      }
      #ifdef printDebugStatements
        Serial.printf("RS485 Message Received: %s\n", (char*)buffer);
      #endif
    }
    else
    {
      #ifdef printDebugStatements
        Serial.printf("No RS485 Response received\n");
      #endif
    }

    vTaskDelayUntil( &startTime, 250 );
  }
}


/*
void TaskDisplay( void *pvParameters )
{
  float localInputs[9] = {}, lastInputs[9] = {}, lastText[11] = {};
  const uint16_t dotSize = 12, halfDot = dotSize / 2,
                 xRect1 = 30, xRect2 = 190, yRect = 50,
                 boxLength = 100, halfLength = boxLength / 2;
  uint32_t stickTimer = 0, textTimer = 0;
  char newText[10] = {}, oldText[10] = {};

  lastText[timeout] = -1; //ensure initial drawing of controller status
  lastText[5] = -1.11; //ensure initial drawing of wheel speed
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
  tft.printf("Test Wheel:");
  tft.setCursor(0, 193);
  tft.printf("Speed Limit:");
  tft.setCursor(0, 216);
  tft.printf("Ramp Time:");

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
      tftRewrite(oldText, newText, 160, 170, 2);

      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%.0f%%", speedLimiter * 100);
      sprintf(oldText, "%.0f%%", lastText[9] * 100);
      tftRewrite(oldText, newText, 160, 193, 2);

      memset(newText, 0, sizeof(newText));
      memset(oldText, 0, sizeof(oldText));
      sprintf(newText, "%.2f", rampTime);
      sprintf(oldText, "%.2f", lastText[10]);
      tftRewrite(oldText, newText, 160, 216, 2);

      memcpy(lastText,  localInputs, sizeof(lastText));
      lastText[9] = speedLimiter;
      lastText[10] = rampTime;
    }

    if (millis() > stickTimer + 100)
    {
      stickTimer = millis();
      if( (lastInputs[lxStick] != localInputs[lxStick]) || (lastInputs[lyStick] != localInputs[lyStick]) )
      {
        tft.fillRect(xRect1 + halfLength + 1 + (halfLength * lastInputs[lxStick]), yRect + halfLength + 1 - (halfLength * lastInputs[lyStick]), dotSize, dotSize, ILI9341_BLACK);
        tft.fillRect(xRect1 + halfLength + 1 + (halfLength * localInputs[lxStick]), yRect + halfLength + 1 - (halfLength * localInputs[lyStick]), dotSize, dotSize, ILI9341_BLUE);
      }

      if( (lastInputs[rxStick] != localInputs[rxStick]) || (lastInputs[ryStick] != localInputs[ryStick]) )
      {
        tft.fillRect(xRect2 + halfLength + 1 + (halfLength * lastInputs[rxStick]), yRect + halfLength + 1 - (halfLength * lastInputs[ryStick]), dotSize, dotSize, ILI9341_BLACK);
        tft.fillRect(xRect2 + halfLength + 1 + (halfLength * localInputs[rxStick]), yRect + halfLength + 1 - (halfLength * localInputs[ryStick]), dotSize, dotSize, ILI9341_BLUE);
      }
      memcpy(lastInputs,  localInputs, sizeof(localInputs));
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void tftRewrite(char * oldString, char * newString, int x, int y, int textSize)
{
  tft.setTextSize(textSize);
  int textLength = 0;

  if(strlen(oldString) >= strlen(newString))
    textLength = strlen(oldString);
  else
    textLength = strlen(newString);
  
  for(int i = 0; i < textLength; i++)
  {
    if(oldString[i] != newString[i])
    {
      if(oldString[i] != NULL)
      {
        tft.setCursor(x+i*textSize*6, y);
        tft.setTextColor(ILI9341_BLACK);
        tft.printf("%c", oldString[i]);
      }
  
      if(newString[i] != NULL)
      {
        tft.setCursor(x+i*textSize*6, y);
        tft.setTextColor(ILI9341_WHITE);
        tft.printf("%c", newString[i]);
      }
    }
  }
}*/

void motionControl()
{
  float inputs[5] = {0, 0, 0, 0, 0};
  float wheelOutput[4] = {0, 0, 0, 0};
  float wheelUse[4] = {0, 0, 0, 0};
  float sendData[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  /***~~~ Check if controller connected, read input from controller ~~~***/
  if (controllerSelected == Playstation)
  {
    readPS4Input(inputs);
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
      Serial.println("No controller connected!");
    #endif
  }
  /********** End FailSafe *************/

  /*~~~ Determine wheel speed from inputs ~~~*/
  if (controllerSelected == RC)
  {
    //wheelUse[Front] = inputs[rxStick] + inputs[lxStick];
    //wheelUse[Right] = -inputs[ryStick] + inputs[lxStick];
    //wheelUse[Back] = -inputs[rxStick] + inputs[lxStick];
    //wheelUse[Left] = inputs[ryStick] + inputs[lxStick];
    wheelUse[Front] = inputs[lyStick];

  }

  if (controllerSelected == Playstation)
  {
    wheelUse[Front] = inputs[lxStick] + inputs[rxStick];
    wheelUse[Right] = -inputs[lyStick] + inputs[rxStick];
    wheelUse[Back] = -inputs[lxStick] + inputs[rxStick];
    wheelUse[Left] = inputs[lyStick] + inputs[rxStick];
  }

  #ifdef printDebugStatements
    Serial.printf("Wheels from input: %f  %f  %f  %f\n", wheelUse[0], wheelUse[1], wheelUse[2], wheelUse[3]);
  #endif

  /*~~~ Create deadzone for input so platform doesnt move unexpectedly ~~~*/
  if ( (abs(wheelUse[Front]) < .1) &&
       (abs(wheelUse[Right]) < .1) &&
       (abs(wheelUse[Back]) < .1) &&
       (abs(wheelUse[Left]) < .1) )
  {
    #ifdef printDebugStatements
      Serial.printf("Controller deadzone\n");
    #endif
    
    for (int i = 0; i < 4; i++)
    {
      wheelUse[i] = 0;
    }

    /*~~~ If controller in deadzone and platform stopped, disable PID control and force output to zero ~~~*/
    if( (usePID == 1) &&
        (abs(scaledWheelSpeed[Front]) < .1) &&
        (abs(scaledWheelSpeed[Right]) < .1) &&
        (abs(scaledWheelSpeed[Back]) < .1) &&
        (abs(scaledWheelSpeed[Left]) < .1) )
    {
      for (int i = 0; i < 4; i++)
      {
        pidControllers[i]->SetMode(MANUAL);
        output[i] = 0;
      }
    }
  }
  else
  {
    if( usePID == 1 )
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
        Serial.printf("Wheel %i exceeds maximum\n", i);
      #endif

      for (int j = i + 1; j != i; j = (j + 1) % 4)
      {
        wheelUse[j] = wheelUse[j] / abs(wheelUse[i]);
      }
      wheelUse[i] = wheelUse[i] / abs(wheelUse[i]);
    }
  }

  
  

  if(usePID == 1)
  {
    for (int i = 0; i < 4; i++)
    {
      targetSpeed[i] = wheelUse[i];
      pidControllers[i]->Compute();
      wheelOutput[i] = output[i];
      #ifdef printDebugStatements 
        Serial.printf("PID %d Input: %.4f (-1 to 1) Target: %.2f (-1 to 1) Output: %.2f (%.2f - %.2f) \n", i, scaledWheelSpeed[i], targetSpeed[i], output[i], throttleMin, throttleMax);
      #endif
    }
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
  for (int i = 0; i < 4; i++)
  {
    /*if (wheelUse[i] >= 0)
    {
      digitalWrite(directionPin[i], HIGH);
      ledcWrite(pwmCh[i], abs(wheelUse[i]) * dutyCycle);
    }

    else
    {
      digitalWrite(directionPin[i], LOW);
      ledcWrite(pwmCh[i], abs(wheelUse[i]) * dutyCycle);
    }*/

    if(wheelOutput[i] >= 0)
      {
        ledcDetachPin(backwardPin[i]);
        ledcWrite(pwmCh[i], abs(wheelOutput[i]) * dutyCycle);
        ledcAttachPin(forwardPin[i], pwmCh[i]);
      }

      else
      {
        ledcDetachPin(forwardPin[i]);
        ledcWrite(pwmCh[i], abs(wheelOutput[i]) * dutyCycle);
        ledcAttachPin(backwardPin[i], pwmCh[i]);
      }
  }

  memcpy(&sendData[0], inputs, sizeof(inputs));
  memcpy(&sendData[5], wheelUse, sizeof(wheelUse));

  xQueueOverwrite(displayQ, &sendData);
}

void readPS4Input(float inputs[])
  {
  inputs[lxStick] = (float) PS4.data.analog.stick.lx / 128;
  inputs[lyStick] = (float) PS4.data.analog.stick.ly / 128;
  inputs[rxStick] = (float) PS4.data.analog.stick.rx / 128;
  inputs[timeout] = !PS4.isConnected();

  #ifdef printDebugStatements
    Serial.println("Controller Status");
    if( PS4.isConnected() )
    {
      //printControllerStatus();
      Serial.println("Inputs");
      Serial.println(inputs[lxStick]);
      Serial.println(inputs[lyStick]);
      Serial.println(inputs[rxStick]);
      Serial.println(inputs[timeout]);
    }
  #endif
  }

void readRCInput(float inputs[])
{
  long localChannelDuration[numChannels];
  uint32_t localLastInput;
  float range[numChannels];
  float inputValue[numChannels];

  //portENTER_CRITICAL(&mux); doesnt matter single cycle instruction //disable interrupts before reading volatiles

  for (int i = 0; i < numChannels; i++)
    localChannelDuration[i] = channelDuration[i];

  localLastInput = lastInput;

  //portEXIT_CRITICAL(&mux); // re-enable interrupts as soon as possible

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
    Serial.println("Controller timeout");
#endif

    inputs[timeout] = 1;
  }
  else
  {
#ifdef printDebugStatements
    Serial.println("Controller CONNECTED");
#endif

    inputs[timeout] = 0;
  }

#ifdef printDebugStatements
  Serial.println("Channel Status");
  for (int i = 0; i < numChannels; i++)
  {
    Serial.printf("Min: %d   Zero: %d   Max: %d  Current: %d \n",
                  channelMin[i], channelZero[i], channelMax[i], localChannelDuration[i]);
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
    //portENTER_CRITICAL(&mux); //disable interrupts before reading volatiles

    for (int i = 0; i < numChannels; i++)
      localChannelDuration[i] = channelDuration[i];

    //portEXIT_CRITICAL(&mux); // re-enable interrupts as soon as possible

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

void modbusMessage( char* address, char* function, char* data)
{
  char buffer[200] = {};
  char hexData[50] = {};
  //char address[3] = "01";
  //char function[3] = "04";
  char checksum[2] = {};

  buffer[0] = ':';

  //stringtohex(address, hexData);
  //strcat(buffer, hexData);
  //Serial.printf("Address hex: %s\n", hexData);
  //memset(hexData, 0, sizeof(hexData));
  strcat(buffer, address);

  //stringtohex(function, hexData);
  //strcat(buffer, hexData);
  //Serial.printf("Function hex: %s\n", hexData);
  //memset(hexData, 0, sizeof(hexData));
  strcat(buffer, function);

  stringtohex(data, hexData);
  strcat(buffer, hexData);

  //Serial.printf("Data hex: %s\n", hexData);
  memset(hexData, 0, sizeof(hexData));

  //Serial.printf("Modbus packet for checksum: %s\n", buffer);

  uint8_t LRC = 0;
  for (int i = 1; i < strlen(buffer); i++)
  {
    LRC -= buffer[i];
  }

  sprintf(hexData, "%02x", LRC);
  strcat(buffer, hexData);
  memset(hexData, 0, sizeof(hexData));

  strcat(buffer, "\r\n");

  #ifdef printDebugStatements
    Serial.printf("Modbus message: %s\n", buffer);
  #endif
  //RS485.write((uint8_t*) buffer, sizeof(buffer));
}

void stringtohex(char* input, char* output)
{
  for (int i = 0; input[i] != '\0'; i++)
  {
    sprintf( (char*)(output + i * 2), "%02x", input[i]);
  }
}

void rampInput(float inputs[])
{
  static uint32_t oldTime = 0, newTime = 0, timeDiff = 0;
  static float currentSpeed[4] = {0, 0, 0, 0};
  float speedDiff;

  if (rampTime == 0) //handle divide by 0
    speedDiff = 1;
  else
    speedDiff = 1 / rampTime / 1000000;

  //static float dt = 1 / rampTime / 1000000;
  newTime = micros();

  if (oldTime == 0)
    oldTime = newTime; // prevent jump from first call

  timeDiff = newTime - oldTime;
#ifdef printDebugStatements
  Serial.printf("Ramp input values -- \nTime Diff: %d ms \ndt: %f \ncurrentSpeeds: %f %f %f %f\n", timeDiff / 1000, speedDiff * 1000, currentSpeed[0],currentSpeed[1],currentSpeed[2],currentSpeed[3]);
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

void IRAM_ATTR RotaryEncoderISR(void)
{  
  //uint32_t timeStart = micros();
  static uint8_t valueRead[4], directionRead[4], valueOld[4];
  
  digitalWrite(2, HIGH); // turn LED on

  for(int i = 0; i < numEncoderChannels; i++)
  {
    valueOld[i] = valueRead[i];
    valueRead[i] = digitalRead(encoderCh[i*2]);

    if (valueRead[i] > valueOld[i])
    {
      encoderDirection[i] = ((digitalRead(encoderCh[i*2+1])*2)-1)*(invertEncoderDirection[i] ? -1 : 1); //sets direction as -1 or 1
    }
    else //if (valueRead < valueOld)
    {
      encoderCount[i]++;
    }
  }

  digitalWrite(2, LOW);

  //lastInterrupt = micros() - timeStart;
}
