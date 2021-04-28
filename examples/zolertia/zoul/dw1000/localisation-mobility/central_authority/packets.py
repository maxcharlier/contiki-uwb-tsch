from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass
from typing import List
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
        return ":".join(map(lambda b: hex(b)[2:], self.address))

    def __hash__(self):
        return hash(self.__str__())

class Packet(ABC):

    type: int

    PACKET_ID_SIZE = {
        ALLOCATION_REQUEST:     PacketParameter(0, 34, 'AllocationRequestPacket'),
        ALLOCATION_SLOT:        PacketParameter(1, 37, 'AllocationSlotPacket'),
        ALLOCATION_ACK:         PacketParameter(2, 16, None),
        DEALLOCATION_REQUEST:   PacketParameter(3, 16, 'DeallocationResquestPacket'),
        DEALLOCATION_SLOT:      PacketParameter(4, 16, None)
    }


    @abstractmethod
    def length(self) -> int:
        pass

    @staticmethod
    def str_to_class(classname):
        return getattr(sys.modules[__name__], classname)


class IncomingPacket(Packet):

    mobile_addr: IPv6Address
    anchor_addr: IPv6Address
    
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

    mobile_addr: IPv6Address
    anchor_addr: IPv6Address
    
    @abstractmethod
    def to_bytearray(self) -> bytearray:
        pass

    @abstractmethod
    def destinations(self) -> List[IPv6Address]:
        pass


class AllocationRequestPacket(IncomingPacket):
    
    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.signal_power = frame[1]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(self, frame[2:18])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(self, frame[18:34])
    
    def length(self):
        return self.PACKET_ID_SIZE[ALLOCATION_REQUEST].size

    def __str__(self):
        return f'AllocationRequestPacket({self.type}, {self.signal_power}, {self.mobile_addr}, {self.anchor_addr})'


class AllocationSlotPacket(OutgoingPacket):

    def __init__(self, mobile_addr: IPv6Address, anchor_addr: IPv6Address, timeslot: int, channel: int = 1):
        assert 0 <= timeslot < 256  # limit timeslot size on one byte
        assert 0 <= channel < 256   # limit channel size on one byte
        
        self.type = self.PACKET_ID_SIZE[ALLOCATION_SLOT].size
        self.mobile_addr: IPv6Address = mobile_addr
        self.anchor_addr: IPv6Address = anchor_addr
        self.timeslot: int = timeslot
        self.channel: int = channel
    
    def to_bytearray(self) -> bytearray:
        frame = bytearray()
        frame.append(self.type)
        frame.extend(self.mobile_addr.address)
        frame.extend(self.anchor_addr.address)
        frame.append(self.timeslot)
        frame.append(self.channel)
        return frame

    
    def destinations(self) -> List[IPv6Address]:
        # TODO: Add anchors in the same geolocalisation cell as self.anchor_addr 
        return [self.mobile_addr, self.anchor_addr]

    def length(self):
        return self.PACKET_ID_SIZE[ALLOCATION_SLOT].size

    def __str__(self):
        return f'AllocationSlotPacket({self.type}, {self.mobile_addr}, {self.anchor_addr}, {self.timeslot}, {self.channel})'


class DeallocationResquestPacket(IncomingPacket):
    
    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(self, frame[1:17])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(self, frame[17:33])
    
    def length(self):
        return self.PACKET_ID_SIZE[DEALLOCATION_REQUEST].size

    def __str__(self):
        return f'DellocationRequestPacket({self.type}, {self.mobile_addr}, {self.anchor_addr})'