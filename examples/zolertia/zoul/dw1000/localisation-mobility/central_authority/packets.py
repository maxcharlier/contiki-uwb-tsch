from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass
import sys

PacketParameter = namedtuple('PacketParameter', 'id size associated_class')

ALLOCATION_REQUEST   = 0
ALLOCATION_SLOT      = 1
ALLOCATION_ACK       = 2
DEALLOCATION_REQUEST = 3
DEALLOCATION_SLOT    = 4

@dataclass
class IPv6Address:
    address: bytearray

    def __str__(self):
        return ":".join(map(lambda x: hex(x)[2:], b))

class Packet(ABC):

    PACKET_ID_SIZE = {
        ALLOCATION_REQUEST:     PacketParameter(0, 34, 'AllocationRequestPacket'),
        ALLOCATION_SLOT:        PacketParameter(1, 16, 'AllocationSlotPacket'),
        ALLOCATION_ACK:         PacketParameter(2, 16, None),
        DEALLOCATION_REQUEST:   PacketParameter(3, 16, None),
        DEALLOCATION_SLOT:      PacketParameter(4, 16, None)
    }


    @abstractmethod
    def length(self) -> int:
        pass

    @staticmethod
    def str_to_class(classname):
        return getattr(sys.modules[__name__], classname)


class IncomingPacket(Packet):
    
    @abstractmethod
    def __init__(self, frame: bytearray):
        pass

    @staticmethod
    def packet_from_bytearray(cls, frame: bytearray):       # -> IncomingPacket
        type = frame[0]
        # type = next(filter(lambda k: PACKET_ID_SIZE[k][0] == size, PACKET_ID_SIZE.keys()))
        type_class = cls.PACKET_ID_SIZE[type].associated_class
        type_class = cls.str_to_class(type_class)
        return type_class(frame)
    
    @staticmethod
    def _parse_ipv6_address(cls, address: bytearray) -> IPv6Address:
        assert len(address) == 16
        return IPv6Address(address)


class OutgoingPacket(Packet):
    
    @abstractmethod
    def to_bytearray(self) -> bytearray:
        pass

class AllocationRequestPacket(IncomingPacket):
    
    def __init__(self, frame: bytearray):
        breakpoint()
        self.size = frame[0]
        self.signal_power = frame[1]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(self, frame[2:18])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(self, frame[18:34])
    
    def length(self):
        return self.PACKET_ID_SIZE[ALLOCATION_REQUEST].size

    def __str__(self):
        return f'AllocationRequestPacket({self.size}, {self.signal_power}, {self.mobile_addr}, {self.anchor_addr})'

class AllocationSlotPacket(OutgoingPacket):

    def __init__(self):
        self.size = self.PACKET_ID_SIZE[ALLOCATION_SLOT].size
