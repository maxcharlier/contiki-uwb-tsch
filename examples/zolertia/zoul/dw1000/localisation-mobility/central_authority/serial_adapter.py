from typing import Dict
import serial
import logging
from time import sleep
import termios
from enum import Enum
from packets import Anchor, IncomingPacket, OutgoingPacket, IPv6Address, ClearSlotframePacket

ADPATERS = {}

PORTS: Dict[IPv6Address, str] = {}

SERIAL_BAUDRATE = 115200

# Byte Stuffing
STATE_WAIT_SFD      = 1
STATE_READ_DATA     = 2
STATE_READ_ESC_DATA = 3

BS_SFD = 0xCC
BS_EFD = 0xEE
BS_ESC = 0x33

DEBUG = False

class SerialAdapter:

    def __init__(self, device: str, clear: bool = False):
        if device in ADPATERS: 
            return ADPATERS[device]

        self.device = device
        self.serial = serial.Serial(
            port=device,
            baudrate=SERIAL_BAUDRATE
        )

        # Flush the uart buffer
        # see https://github.com/gawen947/wsn-tools/blob/ec943642ae5cc488f2c884b8345e36bef583d6a0/uart.c#L140
        sleep(0.5)
        termios.tcflush(self.serial.fd, termios.TCIOFLUSH)

        self.state = STATE_WAIT_SFD
        self.frame = bytearray()


        if clear:
            # Clear the slotframe of the node to start with a clean state.
            self.send(ClearSlotframePacket())
        
        ADPATERS[device] = self


    def flush(self):
        self.serial.flush()
    
    def close(self):
        self.serial.close()

    def send_to(self, pkt: OutgoingPacket):
        '''
        Sends a packet to the correct serial port
        '''

        for dst in pkt.destinations():
            if dst in PORTS:
                port = PORTS[dst]
            else:
                port = Anchor.from_IPv6(dst).USB_port
            
            if port == self.device:
                self.send(pkt)
            else:
                sa = ADPATERS[port]
                sa.send(pkt)
        

    
    def receive(self) -> IncomingPacket:
        '''
        Receives one packet
        '''
        while True:
            recv_data = self.serial.read(1)

            if len(recv_data) == 0:
                # continue
                return

            recv_byte = recv_data[0]
            
            if self.state == STATE_WAIT_SFD:
                if recv_byte == BS_SFD:
                    self.state = STATE_READ_DATA
                else:
                    if DEBUG:
                        logging.debug(f'{self.device}: {recv_byte}')
            elif self.state == STATE_READ_DATA:
                if recv_byte == BS_EFD:
                    logging.info(f'Incomming bytearray from {self.device}: {self.frame}')
                    pkt = IncomingPacket.packet_from_bytearray(self.frame)
                    
                    # Update PORTS to reflect to received packet
                    for p in pkt.origin_addresses():
                        # logging.info(f'Adding {(p, self.device)} to PORTS.')
                        PORTS[p] = self.device
                    
                    # reset state for future use
                    self.state = STATE_WAIT_SFD
                    self.frame = bytearray()
                    return pkt
                elif recv_byte == BS_ESC:
                    self.state = STATE_READ_ESC_DATA
                else:
                    self.frame.append(recv_byte)
            elif self.state == STATE_READ_ESC_DATA:
                self.frame.append(recv_byte)
                self.state = STATE_READ_DATA

    def send(self, pkt: OutgoingPacket):
        send_bytes = self._byte_stuffing_encode(pkt.to_bytearray())
        logging.info(f'Outgoing Packet to {self.device}: {pkt}')
        self.serial.write(send_bytes)

    def _byte_stuffing_encode(self, bytes: bytearray):
        output = bytearray()
        output.append(BS_SFD)
        for b in bytes:
            if b in (BS_SFD, BS_EFD, BS_ESC):
                output.append(BS_ESC)
            output.append(b)
        output.append(BS_EFD)
        return output

