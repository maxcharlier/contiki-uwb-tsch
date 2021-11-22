from __future__ import annotations

import sys
import logging
import math
import csv

from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass
from functools import lru_cache
from typing import List, Dict, Collection, Sized, Union


PacketParameter = namedtuple('PacketParameter', 'id size associated_class')

ALLOCATION_REQUEST   = 0
ALLOCATION_SLOT      = 1
ALLOCATION_ACK       = 2
DEALLOCATION_REQUEST = 3
DEALLOCATION_SLOT    = 4
DEALLOCATION_ACK     = 5
CLEAR_SLOTFRAME      = 6
CLEAR_ACK            = 7
PROPAGATION_TIME     = 8
DEBUGGING            = 255

@dataclass
class IPv6Address:
    address: bytearray

    def __init__(self, address: Union[bytearray, bytes, str]):
        if type(address) is bytearray:
            self.address = address
        elif type(address) is bytes:
            self.address = bytearray(bytes)
        elif type(address) is str:
            self.address = bytearray.fromhex(address)
        else:
            raise ValueError(f"address should be a `bytes` or a `str`, got a {type(address)}")

    def __str__(self):
        return ":".join(map(lambda b: hex(b)[2:], self.address))

    def __eq__(self, o: object) -> bool:
        """
        fe80000000000000fdffffffffff0001 and fd80000000000000fdffffffffff0001 should be the same address
        """
        if type(o) is not IPv6Address:
            return False
        return self.address[3:] == o.address[3:]

    def __hash__(self):
        return hash(self.__str__())

@dataclass
class Anchor:

    address: IPv6Address
    x_pos: float = 0.
    y_pos: float = 0.
    z_pos: float = 0.
    USB_port: str = ""

    def distance(self, other_anchor: Anchor) -> float:
        return math.sqrt(
            (self.x_pos - other_anchor.x_pos) ** 2
            + (self.y_pos - other_anchor.y_pos) ** 2
            + (self.z_pos - other_anchor.z_pos) ** 2
        )
    
    def nearest_anchors(self, amount: int=3) -> List[Anchor]:
        return sorted(self.all_anchors(), key=lambda b: self.distance(b))[:amount]
    
    def __eq__(self, o: object) -> bool:
        if type(o) is not Anchor:
            return False
        return self.address == o.address
    
    def __hash__(self) -> int:
        return self.address.__hash__()

    @classmethod
    def from_IPv6(cls, ipv6: IPv6Address, sourcefile="lab.csv") -> Anchor:
        return next(filter(lambda a: ipv6 == a.address, cls.all_anchors(sourcefile)))
    
    @classmethod
    def all_anchors(cls, sourcefile="lab.csv") -> Collection[Anchor]:
        return list(cls._read_anchors(sourcefile).values())

    @classmethod
    @lru_cache(maxsize=4)
    def _read_anchors(cls, sourcefile="lab.csv") -> Dict[str, Anchor]:
        neighbours: Dict = {}
        with open(sourcefile, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                neighbours[row['node_id']] = Anchor(
                    IPv6Address(row['ipv6']), 
                    float(row['x_pos']), 
                    float(row['y_pos']), 
                    float(row['z_pos']),
                    row['USB_port']
                )
        return neighbours

class Packet(ABC, Sized):

    type: int

    PACKET_ID_SIZE = {
        ALLOCATION_REQUEST:     PacketParameter(0, 16, 'AllocationRequestPacket'),
        ALLOCATION_SLOT:        PacketParameter(1, 16, 'AllocationSlotPacket'),
        ALLOCATION_ACK:         PacketParameter(2, 16, 'AllocationAckPacket'),
        DEALLOCATION_REQUEST:   PacketParameter(3, 16, 'DeallocationResquestPacket'),
        DEALLOCATION_SLOT:      PacketParameter(4, 16, None),
        DEALLOCATION_ACK:       PacketParameter(5, 16, None),
        CLEAR_SLOTFRAME:        PacketParameter(6, 8, 'ClearSlotframePacket'),
        CLEAR_ACK:              PacketParameter(7, 8, 'ClearAckPacket'),
        PROPAGATION_TIME:       PacketParameter(8, 28, 'PropagationTimePacket'),
        DEBUGGING:              PacketParameter(255, 51, 'DebuggingPacket')
    }

    @abstractmethod
    def __len__(self) -> int:
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

    @abstractmethod
    def origin_addresses(self) -> list[IPv6Address]:
        """
        Returns the IPv6 Address of the nodes accessible via this anchor.
        """
        pass

    @classmethod
    def packet_from_bytearray(cls, frame: bytearray) -> IncomingPacket:
        type = frame[0]
        # type = next(filter(lambda k: PACKET_ID_SIZE[k][0] == size, PACKET_ID_SIZE.keys()))
        type_class = cls.PACKET_ID_SIZE[type].associated_class
        type_class = cls.str_to_class(type_class)
        # logging.info(f"type_class: {type_class}")
        return type_class(frame)
    
    @classmethod
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
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(frame[4:20])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(frame[20:36])
    
    def origin_addresses(self) -> Collection[IPv6Address]:
        return [self.mobile_addr, self.anchor_addr]
    
    def __len__(self):
        return self.PACKET_ID_SIZE[ALLOCATION_REQUEST].size

    def __str__(self):
        return f'AllocationRequestPacket({self.signal_power}, {self.mobile_addr}, {self.anchor_addr})'


class AllocationSlotPacket(OutgoingPacket):

    def __init__(self, mobile_addr: IPv6Address, anchor_addr: IPv6Address, timeslot: int, channel: int = 1):
        assert 0 <= timeslot < 256  # limit timeslot size on one byte
        assert 0 <= channel < 256   # limit channel size on one byte
        
        self.type = self.PACKET_ID_SIZE[ALLOCATION_SLOT].id
        self.mobile_addr: IPv6Address = mobile_addr
        self.anchor_addr: IPv6Address = anchor_addr
        self.timeslot: int = timeslot
        self.channel: int = channel
    
    def to_bytearray(self) -> bytearray:
        frame = bytearray()
        frame.append(self.type)
        frame.append(0)                              # ttl
        frame.append(self.timeslot)
        frame.append(self.channel)
        frame.extend(self.mobile_addr.address)
        frame.extend(self.anchor_addr.address)
        return frame

    
    def destinations(self) -> List[IPv6Address]:
        # TODO: Add anchors in the same geolocalisation cell as self.anchor_addr 
        return [self.mobile_addr, self.anchor_addr]

    def __len__(self):
        return self.PACKET_ID_SIZE[ALLOCATION_SLOT].size

    def __str__(self):
        return f'AllocationSlotPacket({self.mobile_addr}, {self.anchor_addr}, {self.timeslot}, {self.channel})'
    
class AllocationAckPacket(IncomingPacket):

    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.timeslot = frame[2]
        self.channel = frame[3]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(frame[4:20])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(frame[20:36])

    def origin_addresses(self) -> Collection[IPv6Address]:
        return [self.mobile_addr, self.anchor_addr]

    def __len__(self):
        return self.PACKET_ID_SIZE[ALLOCATION_ACK].size

    def __str__(self):
        return f'AllocationAckPacket({self.mobile_addr}, {self.anchor_addr}, {self.timeslot}, {self.channel})'


class DeallocationResquestPacket(IncomingPacket):
    
    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.timeslot = frame[2]
        self.channel = frame[3]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(frame[4:20])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(frame[20:36])
    
    def origin_addresses(self) -> Collection[IPv6Address]:
        return [self.mobile_addr, self.anchor_addr]

    def __len__(self):
        return self.PACKET_ID_SIZE[DEALLOCATION_REQUEST].size

    def __str__(self):
        return f'DellocationRequestPacket({self.mobile_addr}, {self.anchor_addr})'

class ClearSlotframePacket(OutgoingPacket):

    def __init__(self):
        self.type = self.PACKET_ID_SIZE[CLEAR_SLOTFRAME].id
    
    def to_bytearray(self) -> bytearray:
        frame = bytearray()
        frame.append(self.type)
        return frame
    
    def destinations(self) -> List[IPv6Address]:
        return []

    def __len__(self):
        return self.PACKET_ID_SIZE[CLEAR_ACK].size

    def __str__(self):
        return f'ClearSlotframePacket({self.type})'



class ClearAckPacket(IncomingPacket):

    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.from_addr: IPv6Address = self._parse_ipv6_address(frame[2:18])
    
    def origin_addresses(self) -> Collection[IPv6Address]:
        return [self.from_addr]

    def __len__(self):
        return self.PACKET_ID_SIZE[CLEAR_SLOTFRAME].size
    
    def __str__(self):
        return f'ClearAckPacket({self.from_addr})'

class PropagationTimePacket(IncomingPacket):

    DWTIME_TO_METRE = 0.0046917639786159
    METRE_TO_DWTIME = 213.139451293

    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.channel = frame[3]
        self.mobile_addr: IPv6Address = self._parse_ipv6_address(frame[4:20])
        self.anchor_addr: IPv6Address = self._parse_ipv6_address(frame[20:36])
        self.prop_time: int = int.from_bytes(frame[36:40], byteorder='little', signed=True)
    
    def origin_addresses(self) -> Collection[IPv6Address]:
        return [self.mobile_addr, self.anchor_addr]
    
    def distance(self) -> float:
        
        return self.prop_time * self.DWTIME_TO_METRE
    
    def __len__(self) -> int:
        return self.PACKET_ID_SIZE[PROPAGATION_TIME].size

    def __str__(self) -> str:
        return f'PropagationTimePacket({self.mobile_addr} -> {self.anchor_addr},  {self.distance()}m)'


class DebuggingPacket(IncomingPacket):

    def __init__(self, frame: bytearray):
        self.type = frame[0]
        self.message = frame[1:50] # todo: decode to string.

    def origin_addresses(self) -> list[IPv6Address]:
        return []

    def __len__(self):
        return self.PACKET_ID_SIZE[DEBUGGING].size
    
    def __str__(self) -> str:
        return f'DebuggingPacket({self.message})'