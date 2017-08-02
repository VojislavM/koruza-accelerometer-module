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

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

#define N 128    // Sampling point

#define RANGE_0_TO_1  0
#define RANGE_1_TO_3  1
#define RANGE_3_TO_6  2
#define RANGE_6_TO_10 3

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

double samplingFrequency = 30;
unsigned int delayTime = 0;

unsigned long previousMillis = 0;        // will store last time accelerometer values are collected

int sample_counter = 0;

int16_t ax, ay, az;

// real values
double ax_in[N];
double ay_in[N];
double az_in[N];

// imag values
double vImag_x[N];
double vImag_y[N];
double vImag_z[N];

// peak values
double peak_x;
double peak_y;
double peak_z;

// average values
double average_sum_x = 0;
int average_count_x = 0;
double average_sum_y = 0;
int average_count_y = 0;
double average_sum_z = 0;
int average_count_z = 0;

// send average valies
double peak_send_x;
double peak_send_y;
double peak_send_z;

double average_0_x = 0;
double average_1_x = 0;
double average_2_x = 0;
double average_3_x = 0;
double max_0_x = 0;
double max_1_x = 0;
double max_2_x = 0;
double max_3_x = 0;

double average_0_y= 0;
double average_1_y = 0;
double average_2_y = 0;
double average_3_y = 0;
double max_0_y = 0;
double max_1_y = 0;
double max_2_y = 0;
double max_3_y = 0;

double average_0_z = 0;
double average_1_z = 0;
double average_2_z = 0;
double average_3_z = 0;
double max_0_z = 0;
double max_1_z = 0;
double max_2_z = 0;
double max_3_z = 0;

#define LED_1 PB3
#define LED_2 PB4
bool blinkState = false;

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

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

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
  /*
  // use the code below to change accel/gyro offset values
  delay(1000);
  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.getAcceleration(&ax, &ay, &az);
  
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.println();

  accelgyro.getAcceleration(&ax, &ay, &az);

  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.println();
  
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
  accelgyro.setXAccelOffset(ax);
  //accelgyro.setYAccelOffset(0);
  //accelgyro.setZAccelOffset(0);
  
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");

  accelgyro.getAcceleration(&ax, &ay, &az);
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.println();
  
  while(1);
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
while(1){  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= delayTime) {
    // save the last time accel data is collected
    previousMillis = currentMillis;

    accelgyro.getAcceleration(&ax, &ay, &az);
    ax_in[sample_counter] = (double)ax;
    ay_in[sample_counter] = (double)ay;
    az_in[sample_counter] = (double)az;
    vImag_x[sample_counter] = 0;
    vImag_y[sample_counter] = 0;
    vImag_z[sample_counter] = 0;

    sample_counter++;
  }

  /* process the collected data */
  if(sample_counter == N){
    sample_counter = 0;
    
    Serial.println("Data:");
    PrintVector(ax_in, ay_in, az_in, N, SCL_TIME);
    Serial.println();
    Serial.println();
  
    //Serial.println("v1");
    FFT.Compute(ax_in, vImag_x, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
    FFT.Compute(ay_in, vImag_y, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
    FFT.Compute(az_in, vImag_z, (uint16_t)N, FFT_FORWARD); /* Compute FFT */
    
    Serial.println("Computed Real values:");
    PrintVector(ax_in, ay_in, az_in, N, SCL_INDEX);
    Serial.println();
    //Serial.println("v2");
    
    FFT.ComplexToMagnitude(ax_in, vImag_x, (uint16_t)N); /* Compute magnitudes */
    FFT.ComplexToMagnitude(ay_in, vImag_y, (uint16_t)N); /* Compute magnitudes */
    FFT.ComplexToMagnitude(az_in, vImag_z, (uint16_t)N); /* Compute magnitudes */
  
    PrintVector(ax_in, ay_in, az_in, (N >> 1), SCL_FREQUENCY); 
    Serial.println();
    Serial.println();

    
  }
} 


  average_0_x = average(ax_in, RANGE_0_TO_1);
  average_1_x = average(ax_in, RANGE_1_TO_3);
  average_2_x = average(ax_in, RANGE_3_TO_6);
  average_3_x = average(ax_in, RANGE_6_TO_10);

  max_0_x = max_value(ax_in, RANGE_0_TO_1);
  max_1_x = max_value(ax_in, RANGE_1_TO_3);
  max_2_x = max_value(ax_in, RANGE_3_TO_6);
  max_3_x = max_value(ax_in, RANGE_6_TO_10);

  average_0_y = average(ay_in, RANGE_0_TO_1);
  average_1_y = average(ay_in, RANGE_1_TO_3);
  average_2_y = average(ay_in, RANGE_3_TO_6);
  average_3_y = average(ay_in, RANGE_6_TO_10);

  max_0_y = max_value(ay_in, RANGE_0_TO_1);
  max_1_y = max_value(ay_in, RANGE_1_TO_3);
  max_2_y = max_value(ay_in, RANGE_3_TO_6);
  max_3_y = max_value(ay_in, RANGE_6_TO_10);

  average_0_z = average(az_in, RANGE_0_TO_1);
  average_1_z = average(az_in, RANGE_1_TO_3);
  average_2_z = average(az_in, RANGE_3_TO_6);
  average_3_z = average(az_in, RANGE_6_TO_10);

  max_0_z = max_value(az_in, RANGE_0_TO_1);
  max_1_z = max_value(az_in, RANGE_1_TO_3);
  max_2_z = max_value(az_in, RANGE_3_TO_6);
  max_3_z = max_value(az_in, RANGE_6_TO_10);

  if(command_received != true){
    average_sum_x += peak_x;
    average_count_x++;
    average_sum_y += peak_y;
    average_count_y++;
    average_sum_z += peak_z;
    average_count_z++;
  }else{
    peak_send_x = average_sum_x / (double)average_count_x;
    peak_send_y = average_sum_y / (double)average_count_y;
    peak_send_z = average_sum_z / (double)average_count_z;
    average_count_x = 0;
    average_count_y = 0;
    average_count_z = 0;

    average_sum_x = 0;
    average_sum_y = 0;
    average_sum_z = 0;
  }

  communicate();
  receive_bytes(rx_buffer, &command_received, &message_len);
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
        //Serial.println("v1");
        
      
//        Serial.print(peak_x);
//        Serial.print(" ");
//        Serial.print(peak_y);
//        Serial.print(" ");
//        Serial.print(peak_z);
//        Serial.println();
      
//        Serial.println("V55");
//        while(1);
       
      //  Serial.print(ax); Serial.print(":");
      //  Serial.print(ay); Serial.print(":");
      //  Serial.println(az);
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_2, blinkState);
        
//        if(peak_x <0.1 || peak_x > 10){
//          acceleroemter_values.ax = 0;
//        }else{
          acceleroemter_values.ax = (int32_t)peak_send_x;
//        }
//        if(peak_y <0.1 || peak_y > 10){
//          acceleroemter_values.ay = 0;
//        }else{
          acceleroemter_values.ay = (int32_t)peak_send_y;
//        }
//        if(peak_z <0.1 || peak_z > 10){
//          acceleroemter_values.az = 0;
//        }else{
          acceleroemter_values.az = (int32_t)peak_send_z;
//        }

        peak_send_x = 0;
        peak_send_y = 0;
        peak_send_z = 0;

//        acceleroemter_values.ax = 10;
//        acceleroemter_values.ay = 20;
//        acceleroemter_values.az = 30;

//
//        gyroscope_values.gx = 40;//gx;
//        gyroscope_values.gy = 50;//gy;
//        gyroscope_values.gz = 60;//gz;
        
        #ifdef OUTPUT_READABLE_ACCELGYRO
            // display tab-separated accel/gyro x/y/z values
            Serial.print("a:\t");
            Serial.print(acceleroemter_values.ax); Serial.print("\t");
            Serial.print(acceleroemter_values.ay); Serial.print("\t");
            Serial.println(acceleroemter_values.az);
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

double average(double *in, int range){
  int start_num = 0;
  int stop_num = 0;
  double average = 0;
  switch(range){
    case 0:
      start_num = 0;
      stop_num = 6; 
      break;
    case 1:
      start_num = 6;
      stop_num = 14; 
      break;
    case 2:
      start_num = 14;
      stop_num = 27; 
      break;
    case 3:
      start_num = 27;
      stop_num = 44; 
      break;
  }
  for(int i = start_num; i < stop_num; i++){
     average += in[i];
  }
  average = average/(stop_num - start_num);
  return average;
}

double max_value(double *in, int range){
  int start_num = 0;
  int stop_num = 0;
  double max_value = 0;
  switch(range){
    case 0:
      start_num = 0;
      stop_num = 6; 
      break;
    case 1:
      start_num = 6;
      stop_num = 14; 
      break;
    case 2:
      start_num = 14;
      stop_num = 27; 
      break;
    case 3:
      start_num = 27;
      stop_num = 44; 
      break;
  }
  for(int i = start_num; i < stop_num; i++){
     max_value = max(max_value, in[i]);
  }
  return max_value;
}


void PrintVector(double *vData, double *vData1, double *vData2, uint8_t bufferSize, uint8_t scaleType){
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
    Serial.print(abscissa, 4);
    Serial.print(" ");
    Serial.print(vData[i]);
    Serial.print(" ");
    Serial.print(vData1[i]);
    Serial.print(" ");
    Serial.print(vData2[i]);
    Serial.println();
  }
  Serial.println();
}
