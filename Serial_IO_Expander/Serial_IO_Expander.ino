/*************************************************
   Band on the Run
   9 November 2020

   Serial IO Expander
   Version 1.0

   By Benjamin Odell

   IO Expander
   1.0
   Configured ESP32 to act as additional IO with software defined timeout
   and checksum to ensure correct output.

*/

HardwareSerial IOExpander(1);

/*~~~~~~~~ Define Variables for PWM channels ~~~~~~~~*/
const int pwmCh[] = {0, 1, 2, 3};     //Define channels for internal ESP32 PWM generator
const int freq = 20000;
const int resolution = 12;
const int dutyCycle = pow(2, resolution);

const int directionPin[] = {25, 27, 21, 17}; //Define pins for motor direction control
const int speedPin[] = {26, 14, 19, 16};  //Define pins for motor speed control

const char local_checksum[] = "85NtAb5Dv8f7pcmb";

void output(float input[4]);

void setup() 
{

  Serial.begin(115200);
  

  IOExpander.begin(400000, SERIAL_8N1, 22, 23);

  for (int i = 0; i < 4; i++)
  {
    pinMode(directionPin[i], OUTPUT);
    ledcSetup(pwmCh[i], freq, resolution);
    ledcWrite(pwmCh[i], 0); //initialize duty cycle to 0
    ledcAttachPin(speedPin[i], pwmCh[i]);
  }

  Serial.printf("Band on the Run \nESP32 IO Expander\n");
  
}

void loop() 
{
  static TickType_t printTick;
  uint8_t buffer[256] = {};
  char checksum[20] = {};
  float decoded[5] = {}, zeroOutput[] = {0, 0, 0, 0};
  int i = 0;
  uint32_t waitStart;
  
  waitStart = micros();

  IOExpander.flush();
  
  //Serial.printf("\nWaiting for communication...\n");

  //Serial.printf("Rx buffer waiting length: %d at %d\n", IOExpander.available(), micros());
  while ( IOExpander.available() < 256 )
  {
    if( (micros() - waitStart) > 100000 )
    {
      output(zeroOutput);
      Serial.printf("Timeout \n");
      //vTaskDelay(50);
    }
  }
  //Serial.printf("Rx buffer reading length: %d at %d\n", IOExpander.available(), micros());
  
  while ( IOExpander.available() > 0 )
  {
    buffer[i++] = IOExpander.read();
  }
  
  sscanf((char *) buffer, "%*s %f %*s %f %*s %f %*s %f %*s %s", &decoded[0], &decoded[1], &decoded[2], &decoded[3], checksum);
 
  if( !strcmp(checksum, local_checksum) )
  {
    output(decoded);
    //Serial.printf("Checksum good\n");
  }
  else
  {
    output(zeroOutput);
    Serial.printf("Checksum bad\n");
  }
   
  
  if(xTaskGetTickCount() - printTick > 1000)
  {
    Serial.printf("\n-------- Data received --------\n");

    Serial.println((char *) buffer);

    Serial.printf("-------- End Data --------\n");
    
    for(int i = 0; i<4; i++)
    {
      Serial.printf("Read value %d: %f\n", i, decoded[i]);
    }
    Serial.printf("Checksum: %s\n", checksum);
    Serial.printf("--------End Message---------\n");

    printTick = xTaskGetTickCount();
  }
}


void output(float input[4])
{
  for (int i = 0; i < 4; i++)
  {
    if (input[i] >= 0)
    {
      digitalWrite(directionPin[i], LOW);
      ledcWrite(pwmCh[i], abs(input[i]) * dutyCycle);
     
    }

    else
    {
      digitalWrite(directionPin[i], HIGH);
      ledcWrite(pwmCh[i], abs(input[i]) * dutyCycle);
    }
  }
}
