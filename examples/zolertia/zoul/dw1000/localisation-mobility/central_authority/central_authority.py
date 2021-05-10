#!/usr/bin/python3


import argh
import logging
from typing import List, Tuple

from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter


def main(*devices: Tuple[str]):

    adapters: List[SerialAdapter] = [SerialAdapter(device, clear=True) for device in devices]
    scheduler = GreedyScheduler(max_length=100)
        
    while True:
        for sa in adapters:

            pkt = sa.receive()
            
            if pkt != None:
                logging.info(f'Incoming packet: {pkt}')
                
                actions = scheduler.schedule(pkt)

                for act in actions:
                    sa.send_to(act)


if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s: %(message)s',
        datefmt='%Y.%m.%d-%H:%M:%S',
        level=logging.DEBUG
    )
    argh.dispatch_command(main)
