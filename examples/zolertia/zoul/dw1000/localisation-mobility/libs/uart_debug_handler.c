#include "contiki.h"
#include "dev/uart.h"
#include "assert.h"

#include <stdio.h>
#include <string.h>

#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/uart_degub_handler.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/send-messages.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"

static int state = STATE_WAIT_SFD;

#define nb_uart_buffer 2
// the first byte of the receive buffer store the lenght of the buffer
uint8_t uart_debug_receive_buffer0[MAX_SERIAL_LEN+1]; 
uint8_t uart_debug_receive_buffer1[MAX_SERIAL_LEN+1];
static uint8_t current_uart_buffer_index = 0;
static uint8_t *receive_ptr[2] = {uart_debug_receive_buffer0, uart_debug_receive_buffer1};
static uint8_t *current_receive_prt;

/**
 * Receive individual byte from the serial line, undo the byte stuffing and 
 * a after complession of the reception of a message trigger the 
 * uart_debug_handler_process.
 * Use double buffering to allow reception of a second message while 
 * processing the previews message.
 * */
int
uart_receive_byte(unsigned char c)
{
  uint8_t byte = c;

  uart_write_byte(UART_DEBUG, byte);

  switch (state){

  case STATE_WAIT_SFD:
    if (byte == BS_SFD) {
      state = STATE_READ_DATA;

      *current_receive_prt = receive_ptr[current_uart_buffer_index];

      memset(current_receive_prt, 0x00 , MAX_SERIAL_LEN+1);
    }
    break;
  
  case STATE_READ_DATA:
    if (byte == BS_EFD) {
      state = STATE_WAIT_SFD;   // TODO move the buffer here as well ?

      // act_on_message(receive_ptr[current_uart_buffer_index]++, receive_ptr - receive_buffer);

      //set the lenght of the buffer in the first byte of the receive_buffer
      *receive_ptr[current_uart_buffer_index] = receive_ptr[current_uart_buffer_index] - current_receive_prt;

      /* Send the PROCESS_EVENT_MSG event asynchronously to 
        "uart_debug_handler_process", with a pointer to the current uart_buffer. */
      process_post(&uart_debug_handler_process,
                      PROCESS_EVENT_MSG, (void *) receive_ptr[current_uart_buffer_index]);

      // Switch to the other buffer
      current_uart_buffer_index = (current_uart_buffer_index +1)%2;

    } else if (byte == BS_ESC) {
      state = STATE_READ_ESC_DATA;
    } else {
      *current_receive_prt++ = byte;
    }
    break;
  
  case STATE_READ_ESC_DATA:
      *current_receive_prt++ = byte;
      state = STATE_READ_DATA;
    break;
  
  default:
    break;
  }
  
  return 1;
}


PROCESS(uart_debug_handler_process, "UART Debug Handler Process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_debug_handler_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();
  current_uart_buffer_index = 0;



  while(1) {

      PROCESS_WAIT_EVENT();

      if(ev == PROCESS_EVENT_MSG){
      	uint8_t lenght = (uint8_t) &data;
      	/* Check if we don't have an memory overflow */ 
      	assert(lenght <= MAX_SERIAL_LEN);

      	act_on_message((uint8_t *) data++, lenght);
      }

  }

  PROCESS_END();
}