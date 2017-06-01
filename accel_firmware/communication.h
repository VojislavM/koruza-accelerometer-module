#ifndef communication_h
#define communication_h

#include "message.h"
//
void receive_bytes(volatile uint8_t *rx_buffer, boolean *cmd_rcv, int *msg_len);
extern bool send_bytes(const message_t *msg_send);
//extern void decodeCommand(const message_t& msg);
//
#endif
