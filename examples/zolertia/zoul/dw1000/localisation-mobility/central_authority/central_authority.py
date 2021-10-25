#!/usr/bin/python3


from struct import pack
import argh
import logging
import threading
from typing import List, Tuple
from queue import Queue

from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter


class Handler:

    def __init__(self, eventQueue: Queue) -> None:
        self.eventQueue = eventQueue

    def handle(self, sa: SerialAdapter):
        while True:
            packet = sa.receive()
            logging.info(f'Incoming packet from {sa.device}: {packet}.')

            self.eventQueue.put((packet, sa.device))
            self.eventQueue.task_done()
        

def main(*devices: Tuple[str]):

    adapters: List[SerialAdapter] = [SerialAdapter(device, clear=True) for device in devices]
    scheduler = GreedyScheduler(max_length=100, offset = 6, serial=adapters[0])  # offset 6 for 2 devices
    eventQueue: Queue = Queue()
    handler = Handler(eventQueue)

    for i in range(len(adapters)):
        t = threading.Thread(target=handler.handle, args=(adapters[i],))
        t.start()

    while True:
        pkt, device = eventQueue.get(block=True)
        # logging.info(f'Handling packet: {pkt} from the queue.')
        actions = scheduler.schedule(pkt, device)

        for act in actions:
            adapters[0].send_to(act)
            


if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s: %(message)s',
        datefmt='%Y.%m.%d-%H:%M:%S',
        level=logging.DEBUG
    )
    argh.dispatch_command(main)
