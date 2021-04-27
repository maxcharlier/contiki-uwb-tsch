#!/usr/bin/python3


import os
import sys
import argh
import datetime
from typing import Union, Tuple

from packets import *
from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter



PACKET_SIZES = {
    0: 36,  # ALLOCIATION_REQUEST
    1: 16,   # ALLOCATION_SLOT
    3: 16   # ALLOCATION_ACK
}





    

def get_packet_size(data: Union[bytearray, bytes]) -> int:
    if type(data) in (bytearray, bytes):
        return PACKET_SIZES[int(data[0])]


def get_packet_from_bytearray(data: bytearray):
    return data

def get_time() -> str:
    time = datetime.datetime.now()
    return str(time)


def main(device: str):
    start_time = get_time()

    serial_adapter = SerialAdapter(device)
        
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


if __name__ == "__main__":    
    argh.dispatch_command(main)
