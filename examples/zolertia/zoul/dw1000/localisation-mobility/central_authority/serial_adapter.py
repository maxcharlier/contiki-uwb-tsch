import serial
import logging
from time import sleep
from enum import Enum
from packets import IncomingPacket, OutgoingPacket


SERIAL_BAUDRATE = 115200

# Byte Stuffing
STATE_WAIT_SFD      = 1
STATE_READ_DATA     = 2
STATE_READ_ESC_DATA = 3

BS_SFD = 0xBB
BS_EFD = 0xEE
BS_ESC = 0x33

class SerialAdapter:

    def __init__(self, device: str):
        self.serial = serial.Serial(
            port=device,
            baudrate=SERIAL_BAUDRATE
        )
    
    def receive(self) -> IncomingPacket:
        '''
        Receives one packet
        '''
        state = STATE_WAIT_SFD
        frame = bytearray()
        
        while True:
            recv_data = self.serial.read(1)
            recv_byte = recv_data[0]
            
            if len(recv_data) == 0:
                continue
            
            if state == STATE_WAIT_SFD:
                if recv_byte == BS_SFD:
                    state = STATE_READ_DATA
            elif state == STATE_READ_DATA:
                if recv_byte == BS_EFD:
                    return IncomingPacket.packet_from_bytearray(IncomingPacket, frame)
                elif recv_byte == BS_ESC:
                    state = STATE_READ_ESC_DATA
                else:
                    frame.append(recv_byte)
            elif state == STATE_READ_ESC_DATA:
                frame.append(recv_byte)
                state = STATE_READ_DATA

    def send(self, pkt: OutgoingPacket):
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

