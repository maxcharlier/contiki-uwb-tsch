#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "dw1000-driver.h"
#include "dw1000.h"

// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(frame_master_process, "Frame master");

AUTOSTART_PROCESSES(&frame_master_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define RIME_CHANNEL 147
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };

#define SIZEOF_QUALITY 14
/** 
 *  We have multiple types of message and there are defined by the value of 
 *  the first byte MODE:
 *    - 0x01: Ranging Request 
 *       A ranging request will be construct as follow: 0x01 SOURCE DEST
 *         Where SOURCE is the initiator of the ranging request
 *         and DEST the receiver of the ranging request
 *    - 0x02: Ranging response
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
 *
 * The physical source (in the header) is used to determine the master node addr
 */


/* used for the ranging exchange */
static uint8_t mode; /* the mode of the received message */
static uint16_t ranging_dest; /* the destination of the ranging request */
static uint16_t master_addr; /* the address of the master node */
static uint16_t tx_delay; /* the tx_delay settings */
static uint16_t rx_delay; /* the rx_delay settings */

/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  if(packetbuf_datalen() >= 1){ 
    PRINTF("Node receive message\n");
    char data[packetbuf_datalen()];
    packetbuf_copyto(data);
    mode = data[0];

    if((mode == 0x01 || mode == 0x02) && packetbuf_datalen() == 5){ 
      /* node receive a ranging request */
      
      uint16_t source, dest;
      source = data[1] | (data[2] << 8);
      dest = data[3] | (data[4] << 8);

      /* The node must be the "source".*/
      if((source & 0xFF) == linkaddr_node_addr.u8[0] && 
           (source >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
          /* we are the source of the ranging request */
          /* Recover the address of the master */
          ranging_dest = dest;
          master_addr = from->u8[0] | (from->u8[1] << 8);
          process_poll(&frame_master_process);
        }
    } 
    else if((mode == 0x01 || mode == 0x02) && packetbuf_datalen() >= 13){
      PRINTF("Master receive a ranging response form %02X%02X\n", from->u8[1], 
        from->u8[0]);
      /* source, dest and report */

      uint64_t propagation_time = 0;
#if DEBUG
      /* only available for the master*/
      uint16_t source, dest;
      source = data[1] | (data[2] << 8);
      dest = data[3] | (data[4] << 8);
      printf("Propagation time between %.4X %.4X: ", source, dest); 
#endif /* DEBUG */

      memcpy(&propagation_time, &data[5], 8);


      printf("0x%08X", (unsigned int) propagation_time);
      if(mode == 0x02){
        /* get quality */
        dw1000_frame_quality quality ;
        memcpy(&quality, &data[13], SIZEOF_QUALITY);
        print_receive_quality(quality);
      }
      else{
        printf("\n");
      }

    }
    else if(mode == 0x03 && packetbuf_datalen() == 5){
      PRINTF("Node receive antenna delay settings form %02X%02X\n", from->u8[1], 
        from->u8[0]);
      /* source, dest and report */
      /* only available for the master*/
      tx_delay = data[1] | (data[2] << 8);
      rx_delay = data[3] | (data[4] << 8);  

      process_poll(&frame_master_process);
    }
    else if(mode == 0x04 && packetbuf_datalen() == 1){
      PRINTF("Node receive antenna delay request form %02X%02X\n", from->u8[1], 
        from->u8[0]);
      /* we need the master addr */
      master_addr = from->u8[0] | from->u8[1] << 8;
      process_poll(&frame_master_process);
    } 
    else if(mode == 0x05 && packetbuf_datalen() == 5){
      PRINTF("Master receive an antenna delay request response form %02X%02X\n",
        from->u8[1], from->u8[0]);
      /* source, dest and report */
      /* only available for the master*/
      uint16_t tx_delay, rx_delay;
      tx_delay = data[1] | (data[2] << 8);
      rx_delay = data[3] | (data[4] << 8);
      PRINTF("Delay antenna from %02X%02X: ", from->u8[1], from->u8[0]);          
      printf("%d %d\n", (unsigned int) tx_delay, (unsigned int) rx_delay);
    }
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
 
  unicast_open(&uc, RIME_CHANNEL, &uc_cb);

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

            dw1000_driver_ranging_request(); 
            packetbuf_copyfrom("", 0);
            unicast_send(&uc, &addr);
            PRINTF("Propagation time between %.4X %.4X: ", source, dest);

            /* wait for the ranging response */
            while(dw1000_driver_is_ranging_request()){
              PROCESS_PAUSE();
            }
            printf("0x%08X ", 
              (unsigned int) dw1000_driver_get_propagation_time());

            if(mode == 0x02){
              /* get quality */
              print_receive_quality(dw1000_driver_get_packet_quality());
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
            char report[5]; // 1 for mode, 2 for source, 2 for dest
            /* store the mode */
            report[0] = mode;
            /* source */
            report[1] = source & 0xFF;
            report[2] = (source >> 8) & 0xFF;
            /* dest */
            report[3] = dest & 0xFF;
            report[4] = (dest >> 8) & 0xFF;
            packetbuf_copyfrom(report, 5);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

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
          PRINTF("tx_delay: %.4X %d\n", tx_delay, (unsigned int) tx_delay);
          PRINTF("rx_delay: %.4X %d\n", rx_delay, (unsigned int) rx_delay);

          if((dest & 0xFF) == linkaddr_node_addr.u8[0] && 
             (dest >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
            PRINTF("Master apply the delay value\n");
            /* Master are the destination of the delay setter */

            dw_set_tx_antenna_delay(tx_delay);
            dw_set_rx_antenna_delay(rx_delay);

            PRINTF("tx delay %d\n", (unsigned int) dw_get_tx_antenna_delay());
            PRINTF("rx delay %d\n", (unsigned int) dw_get_rx_antenna_delay());
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
            /* source */
            report[1] = tx_delay & 0xFF;
            report[2] = (tx_delay >> 8) & 0xFF;
            /* dest */
            report[3] = rx_delay & 0xFF;
            report[4] = (rx_delay >> 8) & 0xFF;
            packetbuf_copyfrom(report, 5);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

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
            printf("%d ", (unsigned int) dw_get_tx_antenna_delay());
            printf("%d\n", (unsigned int) dw_get_rx_antenna_delay());

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
    }
    else if(ev == PROCESS_EVENT_POLL){
      if(mode == 0x01 || mode == 0x02){
        /** 
         * This part is used to make the ranging computation and send the 
         *  report to the master node.
         **/
        PRINTF("Node make ranging\n");
        /* prepare and send the ranging request */
        linkaddr_t dest_addr;
        dest_addr.u8[0]= ranging_dest & 0xFF;
        dest_addr.u8[1]= (ranging_dest >> 8) & 0xFF;

        dw1000_driver_ranging_request(); 
        packetbuf_copyfrom("", 0);
        unicast_send(&uc, &dest_addr);

        /* wait for the ranging response */
        while(dw1000_driver_is_ranging_request()){
          PROCESS_PAUSE();
        }

        /* prepare and  send the ranging report to the master */

        /* 1 for mode, 2 for source, 2 for dest, 8 for report */
        uint frame_size = 13;
        if(mode == 0x02)
          frame_size += SIZEOF_QUALITY;
        char report[frame_size]; 

        report[0] = mode;
        /* source */
        report[1] = linkaddr_node_addr.u8[0];
        report[2] = linkaddr_node_addr.u8[1];
        /* dest */
        report[3] = ranging_dest & 0xFF;
        report[4] = (ranging_dest >> 8) & 0xFF;


        uint64_t propagation = dw1000_driver_get_propagation_time();
        memcpy(&report[5], &propagation, 8);

        if(mode == 0x02){
          /* get quality */
          dw1000_frame_quality quality = dw1000_driver_get_packet_quality();
          memcpy(&report[13], &quality, SIZEOF_QUALITY);
        }

        packetbuf_copyfrom(report, sizeof(report));

        linkaddr_t dest_addr2;
        dest_addr2.u8[0]= master_addr & 0xFF;
        dest_addr2.u8[1]= (master_addr >> 8) & 0xFF;

        /* request an ACK */
        packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

        unicast_send(&uc, &dest_addr2);
      }
      else if(mode == 0x03){
        /** 
         * This part is used to set the delay settings.
         **/
        PRINTF("Node set antenna delay\n");

        dw_set_tx_antenna_delay(tx_delay);
        dw_set_rx_antenna_delay(rx_delay);

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
        /* send the delay configuration to the "master" node */
        linkaddr_t addr;
        addr.u8[0]= master_addr & 0xFF;
        addr.u8[1]= (master_addr >> 8) & 0xFF;

        char report[5]; // 1 for mode, 2 for tx_delay, 2 for rx_delay
        /* store the mode */
        report[0] = 0x05;
        /* source */
        report[1] = tx_delay & 0xFF;
        report[2] = (tx_delay >> 8) & 0xFF;
        /* dest */
        report[3] = rx_delay & 0xFF;
        report[4] = (rx_delay >> 8) & 0xFF;
        packetbuf_copyfrom(report, 5);
        /* request an ACK */
        packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
        unicast_send(&uc, &addr);

        PRINTF("tx_delay: %d\n", (unsigned int) tx_delay);
        PRINTF("rx_delay: %d\n", (unsigned int) rx_delay);
        PRINTF("Antenna delay report sent.\n");
      }
    }
  }
  PROCESS_END();
}
