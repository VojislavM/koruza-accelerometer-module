#include <Arduino.h>
#include "message.h"
#include "frame.h"
#include "communication.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "arduinoFFT.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

#define Fs 3000 // Sampling Frequency
#define N 128    // Sampling point

int16_t ax, ay, az;
int16_t gx, gy, gz;

double ax_in[N];
double ay_in[N];
double az_in[N];

#define LED_1 PB3
#define LED_2 PB4
bool blinkState = false;

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

/*** RECEIVING DATA VARIABLES ***/
/* True when receiving string is completed */
boolean command_received = false;
/* Receiving buffer for incomming serial data */
volatile uint8_t rx_buffer[100];
/* Final length of serial received data */
int message_len = 0;

/* Parsed message */
message_t msg_parsed;
/* Sending message */
message_t msg_send;
/* Command parsed form received message */
tlv_command_t parsed_command;

tlv_acceleroemter_value_t acceleroemter_values;
tlv_gyroscope_value_t gyroscope_values;



#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

double samplingFrequency = 200;
uint8_t amplitude = 100;
unsigned int delayTime = 0;

double vImag_x[N];
double vImag_y[N];
double vImag_z[N];

double peak_x;
double peak_y;
double peak_z;

void setup() {
  delay(1000);
  // begin serial communication
  Serial.begin(115200);
  // Disable debug ports, for standard GPIO use
  disableDebugPorts();
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  #ifdef OUTPUT_READABLE_ACCELGYRO
    // initialize device
    Serial.println("Initializing I2C devices...");
  #endif
  accelgyro.initialize();
  
  #ifdef OUTPUT_READABLE_ACCELGYRO  
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  #endif
  
  // use the code below to change accel/gyro offset values
  /*
  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  */

  if(samplingFrequency<=1000)
    delayTime = 1000/samplingFrequency;
  else
    delayTime = 1000000/samplingFrequency;

  // configure Arduino LED for
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
}

void loop() {
  Serial.println("start");
  for(uint16_t i =0;i<N;i++)
  {
    accelgyro.getAcceleration(&ax, &ay, &az);
    ax_in[i] = (double)ax;
    ay_in[i] = (double)ay;
    az_in[i] = (double)az;
    if(samplingFrequency<=1000)
      delay(delayTime);
    else
      delayMicroseconds(delayTime);
  }
  
  Serial.println("v1");
  FFT.Compute(ax_in, vImag_x, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
  FFT.Compute(ay_in, vImag_y, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
  FFT.Compute(az_in, vImag_z, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
  
  Serial.println("v2");
  FFT.ComplexToMagnitude(ax_in, vImag_x, (uint16_t)N); /* Compute magnitudes */
  FFT.ComplexToMagnitude(ay_in, vImag_y, (uint16_t)N); /* Compute magnitudes */
  FFT.ComplexToMagnitude(az_in, vImag_z, (uint16_t)N); /* Compute magnitudes */

  Serial.println("v3");
  FFT.ComplexToMagnitude(ax_in, vImag_x, (uint16_t)N); /* Compute magnitudes */
  FFT.ComplexToMagnitude(ay_in, vImag_y, (uint16_t)N); /* Compute magnitudes */
  FFT.ComplexToMagnitude(az_in, vImag_z, (uint16_t)N); /* Compute magnitudes */

  Serial.println("v4");
  peak_x = FFT.MajorPeak(ax_in, (uint16_t)N, samplingFrequency);
  peak_y = FFT.MajorPeak(ay_in, (uint16_t)N, samplingFrequency);
  peak_z = FFT.MajorPeak(az_in, (uint16_t)N, samplingFrequency);

  Serial.println("V55");
  while(1);
 
//  Serial.print(ax); Serial.print(":");
//  Serial.print(ay); Serial.print(":");
//  Serial.println(az);
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_2, blinkState);

    
  //communicate();
  //receive_bytes(rx_buffer, &command_received, &message_len);
}

void communicate(void){
  if(command_received){
    frame_parser((uint8_t *)&rx_buffer, message_len, &msg_parsed);
    if (message_tlv_get_command(&msg_parsed, &parsed_command) != MESSAGE_SUCCESS)
    {
      message_free(&msg_parsed);
    }else{
      //message_print(&msg_parsed);
      //Serial.println();
      if (parsed_command == COMMAND_GET_STATUS){
        
        
        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);
        
        acceleroemter_values.ax = peak_x;
        acceleroemter_values.ay = peak_y;
        acceleroemter_values.az = peak_z;

        gyroscope_values.gx = 40;//gx;
        gyroscope_values.gy = 50;//gy;
        gyroscope_values.gz = 60;//gz;
        
        #ifdef OUTPUT_READABLE_ACCELGYRO
            // display tab-separated accel/gyro x/y/z values
            Serial.print("a/g:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_2, blinkState);
            
        message_init(&msg_send);
        message_tlv_add_reply(&msg_send, REPLY_STATUS_REPORT);
        message_tlv_add_acceleroemter_value(&msg_send, &acceleroemter_values);
        //message_tlv_add_gyroscope_value(&msg_send, &gyroscope_values);
        message_tlv_add_checksum(&msg_send);
        send_bytes(&msg_send);
        
        #ifdef OUTPUT_READABLE_ACCELGYRO
          message_print(&msg_send);
          Serial.println();
        #endif
        
        message_free(&msg_send);
        message_free(&msg_parsed);
        
      }else{
        message_free(&msg_parsed);
      }
      command_received = false;
    }
  } 
}



void PrintVector(double *vData, uint8_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / N);
  break;
    }
    Serial.print(abscissa, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.println();
  }
  Serial.println();
}
