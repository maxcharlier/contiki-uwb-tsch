#!/usr/bin/python3


import os
import sys
import argh
import datetime
from typing import Union, Tuple

from packets import *
from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter


def get_time() -> str:
    time = datetime.datetime.now()
    return str(time)


def main(device: str):

    sa = SerialAdapter(device)
        
    while True:
        
        pkt = sa.receive()
        print(pkt)


if __name__ == "__main__":    
    argh.dispatch_command(main)
