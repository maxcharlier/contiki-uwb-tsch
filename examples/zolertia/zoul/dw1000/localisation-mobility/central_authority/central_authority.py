#!/usr/bin/python3


import argh
import logging
import threading
import csv
import time
from typing import Dict, List
from queue import Queue

from scheduler import GreedyScheduler
from serial_adapter import SerialAdapter
from packets import DebuggingPacket, IPv6Address, PropagationTimePacket, Anchor
from multilateration import Coordinates, MultilaterationAlgorithm
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
    propagation_plotter.plot([IPv6Address("fe80000000000000fdffffffffff0001"), 
                              IPv6Address("fe80000000000000fdffffffffff0002"), 
                              IPv6Address("fe80000000000000fdffffffffff0003")])
    geolocation_plotter = GeolocationPlotter(geolocation_csv, False)
    geolocation_plotter.plot(IPv6Address("fe80000000000000fdffffffffff0001"), Coordinates(11.56,9.09))      # For Anchor 4
    geolocation_plotter.plot_xy(IPv6Address("fe80000000000000fdffffffffff0001"), Coordinates(11.56,9.09))      # For Anchor 4
    geolocation_plotter.plot_dist(IPv6Address("fe80000000000000fdffffffffff0001"), Coordinates(11.56,9.09))      # For Anchor 4


def startup_time(*devices: str):
    adapters: List[SerialAdapter] = [SerialAdapter(device, clear=True) for device in devices]
    scheduler = GreedyScheduler(max_length=100, offset = 6, serial=adapters[0])  # offset 6 for 2 devices
    eventQueue: Queue = Queue()
    handler = Handler(eventQueue)

    first_parent_time = None
    first_localisation_time = None

    for i in range(len(adapters)):
        t = threading.Thread(target=handler.handle, args=(adapters[i],))
        t.start()

    while True:
        pkt, device = eventQueue.get(block=True)
        # logging.debug(f'Handling packet: {pkt} from the queue.')

        if isinstance(pkt, DebuggingPacket):
            # The tag changed it's parent.
            if first_parent_time is None:
                first_parent_time = time.time()
                logging.info(f'First parent time: {first_parent_time}')


        if isinstance(pkt, PropagationTimePacket):
            # A propagation packet is received, handle it.

            if first_localisation_time is None:
                first_localisation_time = time.time()
                logging.info(f'First localisation time: {first_localisation_time}')

                if PLOT:
                    with open('startups.csv', 'a') as startups_file:
                        startups_csv = csv.writer(startups_file)
                        startups_csv.writerow([first_parent_time, first_localisation_time])

                # we received all the information we need (first_parent + first_localisation), exit.
                exit(0)

            if pkt.prop_time < 0:
                # issues at the transceiver -> ignore packets with negative propagation time.
                continue
            
            continue

        actions = scheduler.schedule(pkt, device)

        for act in actions:
            adapters[0].send_to(act)

def watch(*devices: str):
    """
    Act as the Central Authority for the specifided devices.
    """

    adapters: List[SerialAdapter] = [SerialAdapter(device, clear=True) for device in devices]
    scheduler = GreedyScheduler(max_length=100, offset = 6, serial=adapters[0])  # offset 6 for 2 devices
    eventQueue: Queue = Queue()
    handler = Handler(eventQueue)

    mlas: Dict[IPv6Address, MultilaterationAlgorithm] = {} # Dict of MultilaterationAlgorithms, one per tag

    if PLOT:
        propagation_plotter = PropagationTimePlotter('propagation.csv')
        geolocation_plotter = GeolocationPlotter('geolocation.csv')

    for i in range(len(adapters)):
        t = threading.Thread(target=handler.handle, args=(adapters[i],))
        t.start()

    while True:
        pkt, device = eventQueue.get(block=True)
        # logging.debug(f'Handling packet: {pkt} from the queue.')

        if isinstance(pkt, PropagationTimePacket):
            # A propagation packet is received, handle it.
            if PLOT: propagation_plotter.write(pkt.anchor_addr, pkt.prop_time)

            if pkt.prop_time < 0:
                # issues at the transceiver -> ignore packets with negative propagation time.
                continue
                
            if pkt.mobile_addr not in mlas:
                mlas[pkt.mobile_addr] = MultilaterationAlgorithm()
            mla = mlas[pkt.mobile_addr]       
            
            coord = mla.gps_solve_on_result(pkt.distance(), Anchor.from_IPv6(pkt.anchor_addr, sourcefile='nodes.csv'))
            if coord is not None:
                logging.info(f"{pkt.mobile_addr} is at {coord}")
                if PLOT: geolocation_plotter.write(pkt.anchor_addr, coord)
            
            continue

        actions = scheduler.schedule(pkt, device)

        for act in actions:
            adapters[0].send_to(act)
            
PLOT = True

if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s: %(message)s',
        datefmt='%Y.%m.%d-%H:%M:%S',
        level=logging.DEBUG
    )
    
    argh.dispatch_commands([watch, plot, startup_time])
