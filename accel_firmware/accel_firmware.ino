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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13
bool blinkState = false;

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

void setup() {
  
   // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  // begin serial communication
  Serial.begin(115200);

   // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

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

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

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
        
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);
        
        acceleroemter_values.ax = ax;
        acceleroemter_values.ay = ay;
        acceleroemter_values.az = az;

        gyroscope_values.gx = gx;
        gyroscope_values.gy = gy;
        gyroscope_values.gz = gz;
        
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
        digitalWrite(LED_PIN, blinkState);
            
        message_init(&msg_send);
        message_tlv_add_reply(&msg_send, REPLY_STATUS_REPORT);
        message_tlv_add_acceleroemter_value(&msg_send, &acceleroemter_values);
        message_tlv_add_gyroscope_value(&msg_send, &gyroscope_values);
        message_tlv_add_checksum(&msg_send);
        send_bytes(&msg_send);

        message_print(&msg_send);
        Serial.println();
        
        message_free(&msg_send);
        message_free(&msg_parsed);
        
      }else{
        message_free(&msg_parsed);
      }
      command_received = false;
    }
  } 
}
