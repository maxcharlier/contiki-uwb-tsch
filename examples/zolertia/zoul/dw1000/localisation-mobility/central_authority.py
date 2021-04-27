#!/usr/bin/python3

import serial
import os
import sys
import datetime
from time import sleep
from typing import Union, Tuple
from abc import ABC, abstractmethod


# Byte Stuffing
BS_SFD = 0xBB
BS_EFD = 0xEE
BS_ESC = 0x33

PACKET_SIZES = {
    0: 36,  # ALLOCIATION_REQUEST
    1: 16,   # ALLOCATION_SLOT
    3: 16   # ALLOCATION_ACK
}

class Packet(ABC):

    @abstractmethod
    def length(self) -> int:
        pass

class IncomingPacket(Packet):
    
    @abstractmethod
    def __init__(self, binary: bytearray):
        pass

class OutgoingPacket(Packet):
    
    @abstractmethod
    def to_bytearray(self) -> bytearray:
        pass


class AllocationRequestPacket(Packet):

    def __init__(self, binary: bytearray):
        self.size = binary[0]
        self.signal_power = None
        self.mobile_addr = None
        self.anchor_addr = None


class GreedySchedule:
    
    def __init__(self, max_length: int, nb_channels: int = 1):
        assert nb_channels == 1             # Constaint for now
        self.max_length = max_length
        self.nb_channels = nb_channels
        self.slotframe = []
        self.holes_in_slotframe = [] 
        self.cells_per_node = {}

    def ask_for_new_cell(source, destination) -> Tuple[int, int]:
        if len(self.holes_in_slotframe) != 0:
            # Add the communication in a hole of the schedule
            free_timeslot = holes_in_slotframe.pop()
            self.slotframe[free_timeslot] = (source, destination)
            return timeslot, 1
        else:
            # Add the communivation slot at the end of the schedule
            assert len(self.slotframe) < self.max_length - 1
            self.slotframe.append(source, destination)
            return len(self.slotframe)-1, 1
        

    def delete_cell(source, destination, timeslot, channel):
        assert self.slotframe[timeslot] == (source, destination)
        self.slotframe[timeslot] == None
        self.holes_in_slotframe.append(timeslot)


    

def get_packet_size(data: Union[bytearray, bytes]) -> int:
    if type(data) in (bytearray, bytes):
        return PACKET_SIZES[int(bytearray[0])]


def get_packet_from_bytearray(data: bytearray):
    return data

def get_time() -> str:
    time = datetime.datetime.now()
    return str(time)


if __name__ == "__main__":


    port = '/dev/ttyUSB0'
    start_time = get_time()


    if len(sys.argv) > 1:
        port = sys.argv[1]

    ser = serial.Serial(
        port=port,
        baudrate=115200
    )
    
    while True:
        received_data: bytearray = bytearray()
        while True:
            received_byte = ser.read(1)
            print(f"Received byte of type {type(received_byte)}: {received_byte}")
            received_data.extend(received_byte)

            if len(received_data) > 0:
                
                print(received_data)

                packet = get_packet_from_bytearray(received_data)
            
        sleep(0.01)