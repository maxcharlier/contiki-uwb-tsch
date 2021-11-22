import logging
import sched
import threading

from collections import namedtuple
from typing import Dict, Tuple, List

from serial_adapter import SerialAdapter
from packets import *


Task = namedtuple('Task', 'port packet_type arguments')

class GreedyScheduler:
    
    def __init__(self, max_length: int, nb_channels: int = 1, offset: int = 0, serial: SerialAdapter = None):
        assert nb_channels == 1             # Constaint for now
        self.max_length = max_length
        self.nb_channels = nb_channels
        self.offset = offset
        self.slotframe = []
        self.holes_in_slotframe = [] 
        self.cells_per_node = {}

        self.known_devices = set()

        self.sa = serial

        self.scheduler = sched.scheduler()
        self.scheduler_thread = threading.Thread(target=self.process_events)
        

    def schedule(self, in_pkt: IncomingPacket, device: str) -> Collection[OutgoingPacket]:
        decisions: set[OutgoingPacket] = set()

        # Send ClearSlotframePaket if first time we receive a packet
        if device in self.known_devices:
            # decisions.add(ClearSlotframePacket())     # TODO should we send it here or only at startup?
            self.known_devices.add(device)

        # Handle reveived Packet
        if type(in_pkt) is AllocationRequestPacket:

            # Give at least slots with 3 anchors for 2D two-way ranging according to the closest
            # anchors to the parent of the mobile.
            main_parent = Anchor.from_IPv6(in_pkt.anchor_addr)
            for a in main_parent.nearest_anchors(1):
                logging.info(f"Additional parent chosen: {a} for {in_pkt.mobile_addr} around {in_pkt.anchor_addr}")
                
                # Adding a slot for that parent
            
                timeslot, channel = self._ask_for_new_cell(in_pkt.mobile_addr, a.address)
                asp = AllocationSlotPacket(in_pkt.mobile_addr, a.address, timeslot, channel)
                decisions.add(asp)

                # Resend if no ack is received in 2 s.
                self.scheduler.enter(delay=2, priority=1, action=self.resend, argument=(asp,))


        elif type(in_pkt) is DeallocationResquestPacket:
            
            # TODO implement it!
            logging.warn(f'Not implemented, skipping request: {in_pkt}')

        
        elif type(in_pkt) is AllocationAckPacket:
            logging.info(f"Canced re-sending packets to {in_pkt.mobile_addr}")
            events_to_cancel = filter(lambda e: e.action == self.resend 
                                            and e.argument[0].mobile_addr == in_pkt.mobile_addr,
                                    self.scheduler.queue)
        
            # Cancel sending packets again
            map(self.scheduler.cancel, events_to_cancel)
        
        elif type(in_pkt) is ClearAckPacket:
            logging.info(f"Clear Ack Packet received.")
            
            events_to_cancel = filter(lambda e: e.action == self.resend
                                            and e.argument[0].from_addr == in_pkt.from_addr,
                                    self.scheduler.queue)
            
            # Cancel sending packets again
            map(self.scheduler.cancel, events_to_cancel)


        else:
            logging.warn(f'Unsupported Incoming Packet, skipping request: {in_pkt}')

        return decisions

    def process_events(self):
        while True:
            self.scheduler.run(block=True)
            

    def _ask_for_new_cell(self, source: IPv6Address, destination: IPv6Address) -> Tuple[int, int]:
        if len(self.holes_in_slotframe) != 0:
            # Add the communication in a hole of the schedule
            virtual_free_timeslot = self.holes_in_slotframe.pop()
            free_timeslot = virtual_free_timeslot + self.offset
            self.slotframe[virtual_free_timeslot] = (source, destination)
            return free_timeslot, 0
        else:
            # Add the communivation slot at the end of the schedule
            assert len(self.slotframe) < self.max_length - 1
            self.slotframe.append((source, destination))
            virtual_timeslot = len(self.slotframe)-1
            timeslot = virtual_timeslot + self.offset
            return timeslot, 0
        

    def _delete_cell(self, source: IPv6Address, destination: IPv6Address, timeslot, channel):
        virtual_timeslot = timeslot - self.offset
        assert self.slotframe[virtual_timeslot] == (source, destination)
        self.slotframe[virtual_timeslot] == None
        self.holes_in_slotframe.append(virtual_timeslot)


    def resend(self, pkt: OutgoingPacket, device: str = ""):
        self.sa.send_to(pkt)
