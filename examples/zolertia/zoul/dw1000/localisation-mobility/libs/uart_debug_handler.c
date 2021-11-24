#include "contiki.h"
#include "dev/uart.h"
#include "assert.h"

#include <stdio.h>
#include <string.h>

#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/uart_debug_handler.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/send-messages.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"


/* Number of UART receive buffer. */
#define NB_RECV_BUFFER 2

struct uart_recv_buffer {
  int     locked;               /* Set to 1 if buffer has been offloaded and still has to be processed by a protothread. */
  int     len;                  /* The length of the message. */
  uint8_t msg[MAX_SERIAL_LEN];  /* The buffer that contains the message. */
};

static int state = STATE_WAIT_SFD;

/* UART multiple receive buffer to offload received messages to the protothreads.
   The first byte stores the lock state of the buffer.
   The second byte stores the length of the buffer. */
static struct uart_recv_buffer  recv_buffers[NB_RECV_BUFFER];
static struct uart_recv_buffer *recv_buffer;
static uint8_t                 *recv_buffer_ptr;
static uint8_t                  recv_buffer_idx;

/**
 * Receive individual byte from the serial line, undo the byte stuffing and
 * a after complession of the reception of a message trigger the
 * uart_debug_handler_process.
 * Use double buffering to allow reception of a second message while
 * processing the previews message.
 *
 * Handle connection failure, so if we receive an unexepted SFD, we discard the
 * previous message and start the reception from scratch.
 * */
int
uart_receive_byte(unsigned char c)
{
  uint8_t byte = c;

  /* If the current receive buffer is locked. It means that
     the message contained in this buffer has been offloaded
     to a protothread, and is currently or will be processed
     while outside this ISR. Therefore we cannot write in
     this buffer. It also means that the new message we are
     currently receiving on UART will be ignored.

     This would only occur if act_on_message() is really slow.
     In principle it should not occur, but if it does, it means
     you should increase NB_RECV_BUFFER.

     TODO: Provide a mean to signal via the UART_DEBUG line
           when this case arises. */
  if(recv_buffer->locked)
    return 0;

  uart_write_byte(UART_DEBUG, byte);

  switch (state){

  case STATE_WAIT_SFD:
    /* Wait for SFD byte or ignore character. */
    if (byte == BS_SFD) {
      /* Start writing the message in the current buffer.
         We fill it with a sentinel value for debugging purposes. */
      recv_buffer     = &recv_buffers[recv_buffer_idx];
      recv_buffer_ptr = recv_buffer->msg;
      memset(recv_buffer_ptr, 0xAB, MAX_SERIAL_LEN);

      state = STATE_READ_DATA;
    }
    break;

  case STATE_READ_DATA:
    if (byte == BS_EFD) {
      /* Lock the buffer and set the length. */
      recv_buffer->locked = 1;
      recv_buffer->len    = recv_buffer_ptr - recv_buffer->msg;

      /* Send the PROCESS_EVENT_MSG event asynchronously to
        "uart_debug_handler_process", with a pointer to the current uart_buffer. */
      process_post(&uart_debug_handler_process,
                   PROCESS_EVENT_MSG,
                   recv_buffer);

      /* Switch to the next buffer. */
      recv_buffer_idx++;
      recv_buffer_idx %= NB_RECV_BUFFER;

      state = STATE_WAIT_SFD;
    } else if (byte == BS_SFD) {
      /* We receive an unexpected/unescaped SFD so the previous message is corrupted.
         Discard the previous message and this byte and start receive the next message. */
      state = STATE_WAIT_SFD;
      uart_receive_byte(BS_SFD);
    } else if (byte == BS_ESC) {
      state = STATE_READ_ESC_DATA;
    } else {
      *recv_buffer_ptr++ = byte;
    }
    break;

  case STATE_READ_ESC_DATA:
    *recv_buffer_ptr++ = byte;
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

  while(1) {
      PROCESS_WAIT_EVENT();

      if(ev == PROCESS_EVENT_MSG){
        struct uart_recv_buffer *offloaded_recv_buffer = (struct uart_recv_buffer *)data;

        /* Check if we don't have an memory overflow */
        assert(offloaded_recv_buffer->len <= MAX_SERIAL_LEN);

        /* Process the message (this can be long). */
        act_on_message(offloaded_recv_buffer->msg, offloaded_recv_buffer->len);

        /* The message has been processed, this buffer is now free. */
        offloaded_recv_buffer->locked = 0;
      }

  }

  PROCESS_END();
}
