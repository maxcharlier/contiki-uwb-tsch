#!/usr/bin/python3


import argh
import logging
from typing import Union, Tuple

from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter


def main(device: str):

    sa = SerialAdapter(device)
        
    while True:
        
        pkt = sa.receive()
        logging.info(f'Incoming packet: {pkt}')


if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s: %(message)s',
        datefmt='%Y.%m.%d-%H:%M:%S',
        level=logging.DEBUG
    )
    argh.dispatch_command(main)
