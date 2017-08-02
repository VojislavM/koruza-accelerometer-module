#include "Arduino.h"
#include "communication.h"
//
////TLV communication modules

#include "frame.h"
#include "inet.h"
//
//
/*** RECEIVING DATA VARIABLES ***/
/* Receiving buffer for incomming serial data */
/* Temperary receiving buffer for incomming serial data*/
uint8_t rx_data[2];
/* Number of bytes received on serial */
int rx_indx;
/* Check buffer for receiving serual data */
uint8_t rx_last[2] = {0x00, 0x00};

/**
   Handles receivign bytes from UART.
*/
void receive_bytes(volatile uint8_t *rx_buffer, boolean *cmd_rcv, int *msg_len){
  //debugSerial.println("receiving bytes");
  
  while (Serial.available()) {
    rx_data[0] = (uint8_t)Serial.read();
    /* Clear Rx_Buffer before receiving new data */
    if (rx_indx == 0) {
      for (int i = 0; i < 100; i++) rx_buffer[i] = 0;
    }

    /* Start byte received */
    if (rx_data[0] == FRAME_MARKER_START) {
      /* Start byte received in the frame */
      //      //debugSerial.println("rx_last, rx_buff[0]:");
      //      //debugSerial.println(rx_last[0], HEX);
      //      //debugSerial.println(rx_buffer[0], HEX);
      if ((rx_last[0] == FRAME_MARKER_ESCAPE) && (rx_buffer[0] == FRAME_MARKER_START)) {
        rx_buffer[rx_indx++] = rx_data[0];
      }
      /* Real start byte received */
      else if (rx_last[0] != FRAME_MARKER_ESCAPE) {
        rx_indx = 0;
        rx_buffer[rx_indx++] = rx_data[0];

      }
    }
    /* End byte received */
    else if (rx_data[0] == FRAME_MARKER_END) {
      /* End byte received in the frame */
      if (rx_last[0] == FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START) {
        rx_buffer[rx_indx++] = rx_data[0];
      }
      /* Real end byte received */
      else if (rx_last[0] != FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START) {
        rx_buffer[rx_indx++] = rx_data[0];
        *msg_len = rx_indx;
        rx_indx = 0;
        /* Transfer complete, data is ready to read */
        *cmd_rcv = true;
        /* Disable USART1 interrupt */
        //HAL_NVIC_DisableIRQ(USART1_IRQn);
      }
    }
    else {
      if (rx_buffer[0] == FRAME_MARKER_START) {
        rx_buffer[rx_indx++] = rx_data[0];
      }
    }
    /* Store last received byte for ESC check */
    rx_last[0] = rx_data[0];

  }
}

bool send_bytes(const message_t *msg_send)
{
  /*** TRANSMITTING DATA VARIABLES ***/
  /* Sending frame buffer */
  uint8_t send_frame[1000];
  /* Sending frame size */
  ssize_t send_frame_size;

  send_frame_size = frame_message(send_frame, sizeof(send_frame), msg_send);

  if (send_frame_size <= 0) return false;

  //	Serial.print("Serialized protocol message with frame:");
  //	Serial.println();
  Serial.write(send_frame, send_frame_size);
  //	Serial.println();

  return true;
}
