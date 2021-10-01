import logging
import sched
import threading
from collections import namedtuple
from typing import Tuple, List

from serial_adapter import SerialAdapter
from packets import *


Task = namedtuple('Task', 'port packet_type arguments')

class GreedyScheduler:
    
    def __init__(self, max_length: int, nb_channels: int = 1, serial: SerialAdapter = None):
        assert nb_channels == 1             # Constaint for now
        self.max_length = max_length
        self.nb_channels = nb_channels
        self.slotframe = []
        self.holes_in_slotframe = [] 
        self.cells_per_node = {}

        self.known_devices = set()

        self.sa = serial

        self.scheduler = sched.scheduler()
        self.scheduler_thread = threading.Thread(target=self.process_events)
        

    def schedule(self, in_pkt: IncomingPacket, device: str) -> List[OutgoingPacket]:
        decisions: List[OutgoingPacket] = []

        # Send ClearSlotframePaket if first time we receive a packet
        if device in self.known_devices:
            decisions.append(ClearSlotframePacket())
            self.known_devices.add(device)

        # Handle reveived Packet
        if type(in_pkt) == AllocationRequestPacket:
            
            timeslot, channel = self._ask_for_new_cell(in_pkt.mobile_addr, in_pkt.anchor_addr)
            asp = AllocationSlotPacket(in_pkt.mobile_addr, in_pkt.anchor_addr, timeslot, channel)
            decisions.append(asp)

            # Resend if no ack is received in 2 s.
            self.scheduler.enter(delay=2, priority=1, action=self.resend, argument=(asp,))


        elif type(in_pkt) == DeallocationResquestPacket:
            
            # TODO implement it!
            logging.warn(f'Not implemented, skipping request: {in_pkt}')

        
        elif type(in_pkt) == AllocationAckPacket:
            logging.info(f"Canced re-sending packets to {in_pkt.mobile_addr}")
            events_to_cancel = filter(lambda e: e.action == self.resend 
                                            and e.argument[0].mobile_addr == in_pkt.mobile_addr,
                                    self.scheduler.queue)
        
            # Cancel sending packets again
            map(self.scheduler.cancel, events_to_cancel)
        
        elif type(in_pkt) == ClearAckPacket:
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
            free_timeslot = self.holes_in_slotframe.pop()
            self.slotframe[free_timeslot] = (source, destination)
            return free_timeslot, 1
        else:
            # Add the communivation slot at the end of the schedule
            assert len(self.slotframe) < self.max_length - 1
            self.slotframe.append((source, destination))
            return len(self.slotframe)-1, 1
        

    def _delete_cell(self, source: IPv6Address, destination: IPv6Address, timeslot, channel):
        assert self.slotframe[timeslot] == (source, destination)
        self.slotframe[timeslot] == None
        self.holes_in_slotframe.append(timeslot)


    def resend(self, pkt: OutgoingPacket, device: str = ""):
        self.sa.send_to(pkt)


    def execute(self, task: Task):
        pass