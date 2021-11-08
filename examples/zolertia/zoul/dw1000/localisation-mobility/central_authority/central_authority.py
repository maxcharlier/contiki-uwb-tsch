#!/usr/bin/python3


import argh
import logging
import threading
from typing import List, Tuple
from queue import Queue

from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter
from packets import PropagationTimePacket, Anchor
from multilateration import MultilaterationAlgorithm
from plotter import PropagationTimePlotter, GeolocationPlotter


class Handler:

    def __init__(self, eventQueue: Queue) -> None:
        self.eventQueue = eventQueue

    def handle(self, sa: SerialAdapter):
        while True:
            packet = sa.receive()
            logging.info(f'Incoming packet from {sa.device}: {packet}.')

            self.eventQueue.put((packet, sa.device))
            self.eventQueue.task_done()

def plot(*csv_files: str):
    """
    Plot histograms of received progagation times.
    """
    assert len(csv_files) == 2, "Expected two CSV files: propagation and geolocation."
    propagation_csv, geolocation_csv = csv_files

    propagation_plotter = PropagationTimePlotter(propagation_csv, False)
    propagation_plotter.plot(Anchor.from_IPv6("fe80000000000000fdffffffffff0001"))
    geolocation_plotter = GeolocationPlotter(geolocation_csv, False)
    geolocation_plotter.plot(Anchor.from_IPv6("fe80000000000000fdffffffffff0001"))


def watch(*devices: str):
    """
    Act as the Central Authority for the specifided devices.
    """

    adapters: List[SerialAdapter] = [SerialAdapter(device, clear=True) for device in devices]
    scheduler = GreedyScheduler(max_length=100, offset = 6, serial=adapters[0])  # offset 6 for 2 devices
    eventQueue: Queue = Queue()
    handler = Handler(eventQueue)

    mla = MultilaterationAlgorithm()        # TODO: Use one multilateration algorithm per tag.

    for i in range(len(adapters)):
        t = threading.Thread(target=handler.handle, args=(adapters[i],))
        t.start()

    while True:
        pkt, device = eventQueue.get(block=True)
        # logging.debug(f'Handling packet: {pkt} from the queue.')

        if isinstance(pkt, PropagationTimePacket):
            # A propagation packet is received, handle it.
            coord = mla.gps_solve_on_result(pkt.distance(), Anchor.from_IPv6(pkt.anchor_addr))
            if coord is not None:
                logging.info(f"{pkt.mobile_addr} is at {coord}")
            continue

        actions = scheduler.schedule(pkt, device)

        for act in actions:
            adapters[0].send_to(act)
            


if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s: %(message)s',
        datefmt='%Y.%m.%d-%H:%M:%S',
        level=logging.DEBUG
    )
    argh.dispatch_commands([watch, plot])
