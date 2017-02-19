#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "dw1000-driver.h"

#define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(frame_master_process, "Frame master");

AUTOSTART_PROCESSES(&frame_master_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define RIME_CHANNEL 147
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };


/** A ranging request will be construct as follow: SOURCE DEST
 *  Where SOURCE is the initiator of the ranging request
 *      and DEST the receiver of the ranging request
 *
 *  A ranging report will be construct as follow: SOURCE DEST PROPAGASION
 *  Where SOURCE is the initiator of the ranging request (2 bytes)
 *      DEST the receiver of the ranging request (2 bytes)
 *      PROPAGATION is the propagation time (8 bytes)
 * the physical source (in the header) is used to determine the master node addr
 */


/* used for the ranging exchange */
static uint16_t ranging_dest; /* the destination of the ranging request */
static uint16_t master_addr; /* the address of the master node */

/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  int i;
  if(packetbuf_datalen() == 4){ 
    PRINTF("Node receive a ranging request\n");
    /* only source and dest */
    /* place the source and the dest in data */
    char data[4];
    uint16_t source, dest;
    packetbuf_copyto(data);
    source = data[0] | data[1] << 8;
    dest = data[2] | data[3] << 8;

    /* tow cases: you are the source or not*/
    if((source & 0xFF) == linkaddr_node_addr.u8[0] && 
         (source >> 8 & 0xFF) == linkaddr_node_addr.u8[1]){
        /* we are the source of the ranging request */
        /* Recover the address of the master */
        ranging_dest = dest;
        master_addr = from->u8[0] | from->u8[1] << 8;
        process_poll(&frame_master_process);
      }
  } else if(packetbuf_datalen() == 12){
    PRINTF("Master receive a ranging response form %04X%04X\n",
                                            from->u8[1],
                                            from->u8[0]);
    /* source, dest and report */
    /* only available for the master*/
    char report[12]; // 2 for source, 2 for dest, 8 for report
    uint16_t source, dest;
    uint64_t propagation_time = 0;
    packetbuf_copyto(report);
    source = report[0] | report[1] << 8;
    dest = report[2] | report[3] << 8;
    for(i = 0; i < 8; i++){
      propagation_time |= (report[i+4] & 0xFF) >> (8 * i);
    }
    printf("Propagation time between %.4X %.4X: %d\n",
       source, dest, (int) propagation_time);
  }
  else{
    PRINTF("Receive with unsuported size (%d) form %04X%04X\n",
                                            packetbuf_datalen(),
                                            from->u8[1],
                                            from->u8[0]);
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
      uint16_t source = strtol(data, &str, 16);
      uint16_t dest = strtol(str, &str, 16);

      PRINTF("received line: %s\n", (char *)data);

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
          printf("Propagation time between %.4X %.4X: ",
            source, dest);

          /* wait for the ranging response */
          while(dw1000_driver_is_ranging_request()){
            PROCESS_PAUSE();
          }
          
          printf("%d\n", (int) dw1000_driver_get_propagation_time());
        }else
        {
          PRINTF("Master send the ranging request to the source.\n");
          /* send the ranging request to the "source" node */
          linkaddr_t addr;
          addr.u8[0]= source & 0xFF;
          addr.u8[1]= (source >> 8) & 0xFF;
          char report[4]; // 2 for source, 2 for dest
          /* store source and dest */
          /* source */
          report[0] = source & 0xFF;
          report[1] = (source >> 8) & 0xFF;
          /* dest */
          report[2] = dest & 0xFF;
          report[3] = (dest >> 8) & 0xFF;
          packetbuf_copyfrom(report, 4);
          /* request an ACK */
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
          unicast_send(&uc, &addr);
          PRINTF("Ranging request sended.\n");
        }
      }
    }
    else if(ev == PROCESS_EVENT_POLL){
    /** 
     * This part is used to make the ranging computation and send the report to
     * the master node.
     **/
    PRINTF("Node make ranging\n");
    int i;
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
    char report[12]; // 2 for source, 2 for dest, 8 for report
    /* store source and dest */
    /* source */
    report[0] = linkaddr_node_addr.u8[0];
    report[1] = linkaddr_node_addr.u8[1];
    /* dest */
    report[2] = ranging_dest & 0xFF;
    report[3] = (ranging_dest >> 8) & 0xFF;
    /* propagation time*/
    for(i = 0; i < 8; i++){
      report[i+4] = (dw1000_driver_get_propagation_time() << (8*i)) & 0xFF;
    }

    packetbuf_copyfrom(report, 12);

    linkaddr_t dest_addr2;
    dest_addr2.u8[0]= master_addr & 0xFF;
    dest_addr2.u8[1]= (master_addr >> 8) & 0xFF;
    /* request an ACK */
    // packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
    unicast_send(&uc, &dest_addr2);
    }
  }
  PROCESS_END();
}