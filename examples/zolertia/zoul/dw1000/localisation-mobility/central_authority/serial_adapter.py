import serial
import logging
from time import sleep
from enum import Enum
from packets import IncomingPacket, OutgoingPacket, IPv6Address, ClearSlotframePacket

ADPATERS = {}


SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT  = 0.005

# Byte Stuffing
STATE_WAIT_SFD      = 1
STATE_READ_DATA     = 2
STATE_READ_ESC_DATA = 3

BS_SFD = 0xBB
BS_EFD = 0xEE
BS_ESC = 0x33

class SerialAdapter:

    def __init__(self, device: str, clear: bool = False):
        if device in ADPATERS: 
            return ADPATERS[device]

        self.device = device
        self.serial = serial.Serial(
            port=device,
            baudrate=SERIAL_BAUDRATE,
            timeout=SERIAL_TIMEOUT
        )

        self.state = STATE_WAIT_SFD
        self.frame = bytearray()


        if clear:
            # Clear the slotframe of the node to start with a clean state.
            self.send(ClearSlotframePacket())
        
        ADPATERS['device'] = self

    def send_to(self, pkt: OutgoingPacket):
        '''
        Sends a packet to the correct serial port
        '''
        PORTS = {
            IPv6Address(bytearray(b'\xfe\x80\x00\x00\x00\x00\x00\x00\xfd\xff\xff\xff\xff\xff\x00\x01')): '/dev/anchor1',
            IPv6Address(bytearray(b'\xfd\x00\x00\x00\x00\x00\x00\x00\xfd\xff\xff\xff\xff\xff\x00\x01')): '/dev/anchor1',
            IPv6Address(bytearray(b'\xfd\x00\x00\x00\x00\x00\x00\x00\xfd\xff\xff\xff\xff\xff\x00\x02')): '/dev/anchor2',
            IPv6Address(bytearray(b'\x00\x80\x00 m\x13\x00\x00\xb9\x13\x00\x00\xbb\x13\x00\x00')): '/dev/anchor2'
        }

        for dst in pkt.destinations():
            if PORTS[dst] == self.device:
                self.send(pkt)
            else:
                sa = SerialAdapter(PORTS[dst])
                sa.send(pkt)
        

    
    def receive(self) -> IncomingPacket:
        '''
        Receives one packet
        '''
        recv_data = self.serial.read(1)

        if len(recv_data) == 0:
            # continue
            return

        # logging.info(f'{self.device}: {recv_data}')

        recv_byte = recv_data[0]
        
        if self.state == STATE_WAIT_SFD:
            if recv_byte == BS_SFD:
                self.state = STATE_READ_DATA
            else:
                if recv_byte > 255: 
                    logging.info(f'{self.device}: Skipped reading byte: {recv_byte}')
                else:
                    logging.info(f'{self.device}: Skipped reading byte: {recv_byte} / {chr(recv_byte)}')
        elif self.state == STATE_READ_DATA:
            if recv_byte == BS_EFD:
                pkt = IncomingPacket.packet_from_bytearray(IncomingPacket, self.frame)
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
        logging.info(f'Outgoing Packet to {self.device}: {pkt}')
        send_bytes = self._byte_stuffing_encode(pkt.to_bytearray())
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

