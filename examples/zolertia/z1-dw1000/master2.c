#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "dw1000-driver.h"
#include "dw1000-util.h"
#include "dw1000.h"

// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(frame_master_process, "Frame master");
PROCESS(receive_process, "Receive manager");
PROCESS(receive_debug_process, "Receive debug");

AUTOSTART_PROCESSES(&frame_master_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define RIME_CHANNEL 151
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };

#define SIZEOF_QUALITY 16

void set_tr_delay(uint16_t tx_delay, uint16_t rx_delay);
/** 
 *  We have multiple types of message and there are defined by the value of 
 *  the first byte MODE:
 *    - 0x01: Ranging Request  /response
 *       A ranging request will be construct as follow: 0x01 SOURCE DEST
 *         Where SOURCE is the initiator of the ranging request
 *         and DEST the receiver of the ranging request
 *    - 0x02: Ranging Request /response
 *       A ranging report will be construct as follow: 
 *       0x02 SOURCE DEST PROPAGATION QUALITY
 *       Where SOURCE is the initiator of the ranging request (2 bytes)
 *         DEST the receiver of the ranging request (2 bytes)
 *         PROPAGATION is the propagation time (8 bytes)
 *         QUALITY contain some informations for the computation of the RX power 
 *            and the First path power
 *    - 0x03: Set the TX and RX delay, this is construct as follow:
 *       0x03 DEST TX_DELAY RX_DELAY
 *    - 0X04: Get the TX and RX delay, this is construct as follow:
 *       0x04 DEST
 *    - 0X05: Get (response) the TX and RX delay, this is construct as follow:
 *       0x05 DEST TX_DELAY RX_DELAY
 *    - 0X06: Set the TX power, this is construct as follow:
 *       0x06 DEST TX_POWER
 *    - 0X07: Get (response) the TX power, this is construct as follow:
 *       0x07 DEST TX_POWER
 *
 * The physical source (in the header) is used to determine the master node addr
 */


/* used for the ranging exchange */
static uint8_t mode; /* the mode of the received message */
static uint16_t master_addr; /* the address of the master node */
static uint16_t tx_delay; /* the tx_delay settings */
static uint16_t rx_delay; /* the rx_delay settings */
static uint32_t tx_power;
/* store the last receivedd messag*/
static char payload[30];
static uint8_t payload_len;

static uint8_t message_received = 0; /* indicate if a message was received */
static uint8_t message_init = 0; /* indicate if a message was received */

static uint32_t receive_debug = 0; /* indicate if a message was received */

/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  if(packetbuf_datalen() >= 1){ 
    PRINTF("Node receive message\n");
    payload_len = packetbuf_datalen();
    // char payload2[payload_len];
    // payload = payload2; /* realoc the payload pointer */
    packetbuf_copyto(payload);
    mode = payload[0];

    master_addr = from->u8[0] | (from->u8[1] << 8);

    process_poll(&receive_process);

    message_received = 1; 
    message_init = 1;
  }
  else{
    PRINTF("Receive with unsupported size (%d) form %02X%02X\n", 
      packetbuf_datalen(), from->u8[1], from->u8[0]);
  }
}


PROCESS_THREAD(frame_master_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
  PROCESS_BEGIN();

  printf("Node addr %02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("     1 Ranging request:            1 TWR SOURCE DEST\n");
  printf("     2 Ranging and quality request 2 TWR SOURCE DEST\n");
  printf("     TWR 0 for SS TWR, 1 for DS TWR\n");
  printf("     3 Set TX and RX antenna delay 3 DEST TX_DELAY RX_DELAY\n");
  printf("     4 Get TX and RX antenna delay 4 DEST\n");
  printf("     6 Set TX power                5 DEST TX_POWER\n");
  printf("     7 Get TX power                6 DEST\n");
  printf("     9 Print sys status and error counter\n");
  printf("     A Reset error counter\n");
  printf("     B display number of reset\n");

  /* set the antenna delay, we set these values with the antenna set to off */
  dw1000_driver_off();
  // round 1
// [ 32955.18360298,  32979.74060298,  32946.60960298,  32982.25760298,
//         32906.42560298]
  // round 2
  // [ 32921.60834902  32906.65234902  32916.39834902  32943.32034902, 32933.80234902]

  switch(linkaddr_node_addr.u8[0]){
    case 0x6:
      dw_set_antenna_delay(32922);
      break;

    case 0x07:
      dw_set_antenna_delay(32907);
      break;

    case 0x08:
      dw_set_antenna_delay(32916);
      break;

    case 0x09:
      dw_set_antenna_delay(32943);
      break;

    case 0x0A:
      dw_set_antenna_delay(32934);
      break;
  }
  dw1000_driver_on();

  printf("tx delay %u\n", dw_get_tx_antenna_delay());
  printf("rx delay %u\n", dw_get_rx_antenna_delay());

  unicast_open(&uc, RIME_CHANNEL, &uc_cb);

  process_start(&receive_process, NULL);
  // process_start(&receive_debug_process, NULL);

  for(;;) {
    PROCESS_WAIT_EVENT();
    /* master part */
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      mode = strtol(data, &str, 16);

      PRINTF("Received line: %s\n", (char *)data);
      if(mode == 0x01 || mode == 0x02){ /* ranging request */
        uint8_t twr = strtol(str, &str, 16);
        uint16_t source = strtol(str, &str, 16);
        uint16_t dest = strtol(str, &str, 16);
        if(source > 0 && dest > 0){
          PRINTF("source: %.4X\n", source);
          PRINTF("dest: %.4X\n", dest);

          if((source & 0xFF) == linkaddr_node_addr.u8[0] && 
             (source >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master start the ranging\n");
            /* we are the source of the ranging request */
            linkaddr_t addr;
            addr.u8[0]= dest & 0xFF;
            addr.u8[1]= (dest >> 8) & 0xFF;
            if(twr) // twr == 1
              dw1000_driver_sdstwr_request(); 
            else // twr == 0
              dw1000_driver_sstwr_request(); 
            packetbuf_copyfrom("", 0);
            unicast_send(&uc, &addr);
            PRINTF("Propagation time between %.4X %.4X: ", source, dest);

            /* wait for the ranging response */
            while(dw1000_driver_is_ranging_request()){
              PROCESS_PAUSE();
            }
            printf("0x%08" PRIx32 "", 
              (long int) dw1000_driver_get_propagation_time());

            if(mode == 0x02){
              /* get quality */
              print_receive_quality(dw1000_driver_get_packet_quality());
              printf(" 0x%04X 0x%04X\n",
                (DW1000_PRF == DW_PRF_16_MHZ) ? 16 : 64, 
                DW1000_DATA_RATE);
            }
            else{
              printf("\n");
            }
          }else
          {
            PRINTF("Master send the ranging request to the source.\n");
            /* send the ranging request to the "source" node */
            linkaddr_t addr;
            addr.u8[0]= source & 0xFF;
            addr.u8[1]= (source >> 8) & 0xFF;
            char request[6]; // 1 for mode, 2 for source, 2 for dest
            /* store the mode */
            request[0] = mode;
            /* ranging type */
            request[1] = twr;
            /* source */
            memcpy(&request[2], &source, 2);
            /* dest */
            memcpy(&request[4], &dest, 2);

            packetbuf_copyfrom(request, 6);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);

            unicast_send(&uc, &addr);
            PRINTF("Ranging request sended.\n");
          }
        }
      }
      else if(mode == 0x03){ 
        /* Set the TX and RX delay, this is construct as follow:
        *  0x03 DEST TX_DELAY RX_DELAY */
        uint16_t dest = strtol(str, &str, 16);
        tx_delay = strtol(str, &str, 16);
        rx_delay = strtol(str, &str, 16);
        if(dest > 0){
          PRINTF("dest: %.4X\n", dest);
          PRINTF("tx_delay: 0x%.4X %u\n", tx_delay, (unsigned int) tx_delay);
          PRINTF("rx_delay: 0x%.4X %u\n", rx_delay, (unsigned int) rx_delay);

          if((dest & 0xFF) == linkaddr_node_addr.u8[0] && 
             (dest >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master apply the delay value\n");
            /* Master are the destination of the delay setter */

            set_tr_delay(tx_delay, rx_delay);

            PRINTF("tx delay %.4X\n", dw_get_tx_antenna_delay());
            PRINTF("rx delay %.4X\n", dw_get_rx_antenna_delay());
          }else
          {
            PRINTF("Master send the delay value settings.\n");
            /* send the delay settings to the "dest" node */
            linkaddr_t addr;
            addr.u8[0]= dest & 0xFF;
            addr.u8[1]= (dest >> 8) & 0xFF;
            char report[5]; // 1 for mode, 2 for tx_delay, 2 for rx_delay
            /* store the mode */
            report[0] = mode;

            memcpy(&report[1], &tx_delay, 2);
            memcpy(&report[3], &rx_delay, 2);

            packetbuf_copyfrom(report, 5);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);

            unicast_send(&uc, &addr);
            PRINTF("Delay value settings sended.\n");
          }
        }
      }
      else if(mode == 0x04){ 
        /* 0X04: Get the TX and RX delay, this is construct as follow:
         *       0x04 DEST*/
        uint16_t dest = strtol(str, &str, 16);
        if(dest > 0){
          PRINTF("dest: %.4X\n", dest);

          if((dest & 0xFF) == linkaddr_node_addr.u8[0] && 
             (dest >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master get the delay value: ");
            /* Master are the destination of the delay getter */
            printf("0x%.4X ", dw_get_tx_antenna_delay());
            printf("0x%.4X\n", dw_get_rx_antenna_delay());

          }else
          {
            PRINTF("Master send the delay value request.\n");
            /* send the delay settings request to the "dest" node */
            linkaddr_t addr;
            addr.u8[0]= dest & 0xFF;
            addr.u8[1]= (dest >> 8) & 0xFF;
            char report[1]; // 1 for mode
            /* store the mode */
            report[0] = mode;
            packetbuf_copyfrom(report, 1);
            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
            unicast_send(&uc, &addr);
            PRINTF("Delay value request sended.\n");
          }
        }
      }
      else if(mode == 0x06){ 
        /* Set the TX POWER, this is construct as follow:
        *  0x03 DEST TX_POWER */
        uint16_t dest = strtol(str, &str, 16);
        tx_power = strtol(str, &str, 16);
        if(dest > 0){
          PRINTF("dest: %.4X\n", dest);
          PRINTF("tx_delay: 0x%.4X %u\n", tx_delay, (unsigned int) tx_delay);
          PRINTF("rx_delay: 0x%.4X %u\n", rx_delay, (unsigned int) rx_delay);

          if((dest & 0xFF) == linkaddr_node_addr.u8[0] && 
             (dest >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master apply the tx power settings\n");
            /* Master are the destination of the TX power setter */

            dw1000_driver_off();
            dw_change_tx_power(tx_power, 1);
            dw1000_driver_on();
          }else
          {
            PRINTF("Master send the TX POWER value settings.\n");
            /* send the delay settings to the "dest" node */
            linkaddr_t addr;
            addr.u8[0]= dest & 0xFF;
            addr.u8[1]= (dest >> 8) & 0xFF;
            char report[5]; // 1 for mode, 2 for tx_delay, 2 for rx_delay
            /* store the mode */
            report[0] = mode;

            memcpy(&report[1], &tx_power, 4);

            packetbuf_copyfrom(report, 5);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);

            unicast_send(&uc, &addr);
            PRINTF("Delay value settings sended.\n");
          }
        }
      }
      else if(mode == 0x07){ 
        /* 0X04: Get the TX POWER, this is construct as follow:
         *       0x04 DEST*/
        uint16_t dest = 0;
        dest = strtol(str, &str, 16);
        if(dest > 0){
          PRINTF("dest: %.4X\n", dest);

          if((dest & 0xFF) == linkaddr_node_addr.u8[0] && 
             (dest >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master get the TX POWER value: ");
            /* Master are the destination of the delay getter */
            tx_power = dw_get_tx_power();
            printf("0x%.4X%.4X\n", (unsigned int) (tx_power>>16) , 
                  (unsigned int) (tx_power&0xFFFF)) ;

          }else
          {
            PRINTF("Master send the TX POWER value request.\n");
            /* send the delay settings request to the "dest" node */
            linkaddr_t addr;
            addr.u8[0]= dest & 0xFF;
            addr.u8[1]= (dest >> 8) & 0xFF;
            char report[1]; // 1 for mode
            /* store the mode */
            report[0] = mode;
            packetbuf_copyfrom(report, 1);
            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
            unicast_send(&uc, &addr);
            PRINTF("TX POWER value request sended.\n");
          }
        }
      }
      else if(mode == 0x09){ 
        dw_conf_print();
        print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
        print_error_counter();
        print_sys_state(dw_read_reg_64(DW_REG_SYS_STATE, DW_LEN_SYS_STATE));
      }
      else if(mode == 0x0A){ 
        reset_error_counter();
      }
      else if(mode == 0x0B){ 
        printf("Nb reinitialisation of the receiver %u\n",
                  (unsigned int) receive_debug);
      }
      else if(mode == 0x0C){
        printf("Re init RX\n"); 
        dw1000_driver_off();
        dw1000_driver_on();
      }
    }
  }
  PROCESS_END();
}

/* Use to manage a response */
PROCESS_THREAD(receive_process, ev, data)
{

  PROCESS_BEGIN();

  while (1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_POLL){
      if(mode == 0x01 || mode == 0x02){
        if(payload_len == 6){
          uint8_t twr;
          uint16_t source, dest;
          twr = payload[1];
          source = payload[2] | (payload[3] << 8);
          dest = payload[4] | (payload[5] << 8);
          /* The node must be the "source".*/
          if((source & 0xFF) == linkaddr_node_addr.u8[0] && 
          (source >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            /* we are the source of the ranging request */
            /* Recover the address of the master */
            /** 
            * This part is used to make the ranging computation and send the 
            *  report to the master node.
            **/

            PRINTF("Node make ranging\n");
            /* prepare and send the ranging request */
            linkaddr_t dest_addr;
            dest_addr.u8[0]= dest & 0xFF;
            dest_addr.u8[1]= (dest >> 8) & 0xFF;
            if(twr) // twr == 1
              dw1000_driver_sdstwr_request(); 
            else // twr == 0
              dw1000_driver_sstwr_request(); 
            packetbuf_copyfrom("", 0);
            unicast_send(&uc, &dest_addr);

            /* wait for the ranging response */
            while(dw1000_driver_is_ranging_request()){
              PROCESS_PAUSE();
            }

            PRINTF("Node send response\n");
            /* prepare and  send the ranging report to the master */

            /* 1 for mode, 2 for source, 2 for dest, 8 for report */
            uint frame_size = 9;
            if(mode == 0x02)
              frame_size += SIZEOF_QUALITY;
            char report[frame_size]; 

            report[0] = mode;
            /* source */
            report[1] = linkaddr_node_addr.u8[0];
            report[2] = linkaddr_node_addr.u8[1];
            /* dest */
            dest = payload[3] | (payload[4] << 8);
            report[3] = dest & 0xFF;
            report[4] = (dest >> 8) & 0xFF;

            int32_t propagation = dw1000_driver_get_propagation_time();
            memcpy(&report[5], &propagation, 4);

            // printf("propagation 0x%08X ", 
            //   (unsigned int) propagation);

            if(mode == 0x02){
              /* get quality */
              dw1000_frame_quality quality = dw1000_driver_get_packet_quality();
              memcpy(&report[9], &quality, SIZEOF_QUALITY);
            }

            packetbuf_copyfrom(report, sizeof(report));

            linkaddr_t dest_addr2;
            dest_addr2.u8[0]= master_addr & 0xFF;
            dest_addr2.u8[1]= (master_addr >> 8) & 0xFF;

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);

            unicast_send(&uc, &dest_addr2);
            }
        }
        else if(payload_len >= 9){
          PRINTF("Master receive a ranging response form %04X\n", master_addr);
          /* source, dest and report */

          int32_t propagation_time = 0;
          #if DEBUG
            /* only available for the master*/
            uint16_t source, dest;
            source = payload[1] | (payload[2] << 8);
            dest = payload[3] | (payload[4] << 8);
            printf("Propagation time between %.4X %.4X: ", source, dest); 
          #endif /* DEBUG */

          memcpy(&propagation_time, &payload[5], 4);

          printf("0x%08" PRIx32 "", (long int) propagation_time);
          if(mode == 0x02){
            /* get quality */
            dw1000_frame_quality quality ;
            memcpy(&quality, &payload[9], SIZEOF_QUALITY);
            print_receive_quality(quality);
            printf(" 0x%04X 0x%04X\n",
              (DW1000_PRF == DW_PRF_16_MHZ) ? 16 : 64, 
              DW1000_DATA_RATE);
          }
          else{
            printf("\n");
          }
        }
        
      }
      else if(mode == 0x03){
        /** 
         * This part is used to set the delay settings.
         **/
        PRINTF("Node set antenna delay\n");

        memcpy(&tx_delay, &payload[1], 2);
        memcpy(&rx_delay, &payload[3], 2); 

        /* set the tx and rx delay */
        set_tr_delay(tx_delay, rx_delay);

        PRINTF("tx_delay: 0x%.4X %d\n", tx_delay, (unsigned int) tx_delay);
        PRINTF("rx_delay: 0x%.4X %d\n", rx_delay, (unsigned int) rx_delay);
      }
      else if(mode == 0x04){
        /** 
         * This part is used to get the delay settings and send the report to
         * the master node.
         **/
        PRINTF("Node get antenna delay\n");
        uint16_t tx_delay = dw_get_tx_antenna_delay();
        uint16_t rx_delay = dw_get_rx_antenna_delay();


        PRINTF("tx_delay: %d\n", (unsigned int) tx_delay);
        PRINTF("rx_delay: %d\n", (unsigned int) rx_delay);

        /* send the delay configuration to the "master" node */
        linkaddr_t addr;
        addr.u8[0]= master_addr & 0xFF;
        addr.u8[1]= (master_addr >> 8) & 0xFF;

        char report[5]; // 1 for mode, 2 for tx_delay, 2 for rx_delay
        /* store the mode */
        report[0] = 0x05;

        memcpy(&report[1], &tx_delay, 2);
        memcpy(&report[3], &rx_delay, 2);

        packetbuf_copyfrom(report, 5);
        /* request an ACK */
        packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
        unicast_send(&uc, &addr);

        PRINTF("Antenna delay report sent.\n");
      }
      else if(mode == 0x05){
        PRINTF("Master receive an antenna delay request response form %04X\n", master_addr);
        /* only available for the master*/

        memcpy(&tx_delay, &payload[1], 2);
        memcpy(&rx_delay, &payload[3], 2);  

        PRINTF("Delay antenna from %04X: ", master_addr);          
        printf("0x%.4X 0x%.4X\n",  tx_delay, rx_delay);
      }
      else if(mode == 0x06){
        /** 
         * This part is used to set the delay settings.
         **/
        PRINTF("Node set the TX POWER\n");

        memcpy(&tx_power, &payload[1], 4);

        dw1000_driver_off();
        dw_change_tx_power(tx_power, 1);
        dw1000_driver_on();

        PRINTF("TX POWER: 0x%.4X%.4X\n", (unsigned int) (tx_power>>16) , 
                   (unsigned int) (tx_power&0xFFFF));
      }
      else if(mode == 0x07){
        if(payload_len == 5){
          PRINTF("Node receive a TX power response form %04X\n", master_addr);
          /* source, dest and report */
          memcpy(&tx_power, &payload[1], 4);

          PRINTF("TX POWER from %04X\n", master_addr);          
          printf("0x%.4X%.4X\n", (unsigned int) (tx_power>>16) , 
                   (unsigned int) (tx_power&0xFFFF));
        }
        else if(payload_len == 1){
          /** 
           * This part is used to get the TX POWER and send the report to
           * the master node.
           **/
          PRINTF("Node receive a TX power demande form %04X\n", master_addr);
          PRINTF("Node get TX POWER\n");
          tx_power = dw_get_tx_power();

          /* send the delay configuration to the "master" node */
          linkaddr_t addr;
          addr.u8[0]= master_addr & 0xFF;
          addr.u8[1]= (master_addr >> 8) & 0xFF;

          char report[5]; // 1 for mode, 2 for tx_delay, 2 for rx_delay
          /* store the mode */
          report[0] = 0x07;

          memcpy(&report[1], &tx_power, 4);

          packetbuf_copyfrom(report, 5);
          /* request an ACK */
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
          unicast_send(&uc, &addr);

          PRINTF("TX POWER: 0x%.4X%.4X\n", (unsigned int) (tx_power>>16) , 
                   (unsigned int) (tx_power&0xFFFF));
          PRINTF("TX POWER report sent.\n");
        }
      }
    }
  }
  
  PROCESS_END();
}


void set_tr_delay(uint16_t tx_delay, uint16_t rx_delay){
  dw1000_driver_off();
  dw_set_tx_antenna_delay(tx_delay);
  dw_set_rx_antenna_delay(rx_delay);
  dw1000_driver_on();
}

/* use to reset the receiver if no message was received 
    in the last two seconds */
PROCESS_THREAD(receive_debug_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  etimer_set(&timer, CLOCK_SECOND * 2);
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {
      if(!message_received && message_init){
        dw1000_driver_off();
        dw1000_driver_on();
        receive_debug +=1;
      }
      message_received = 0;
      etimer_reset(&timer);
    }
  }
  
  PROCESS_END();
}
