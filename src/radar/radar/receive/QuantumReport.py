from dataclasses import dataclass, field
import struct

@dataclass(frozen=True, order=True)
class QuantumReport:
    type: int
    status: int
    something_1: list[int]
    bearing_offset: int
    something_14: int
    interference_rejection: int
    something_13: list[int]
    range_index: int
    mode: int
    controls: list[int]
    target_expansion: int
    something_9: int
    something_10: list[int]
    mbs_enabled: int
    something_11: list[int]
    ranges: list[int]
    something_12: list[int]
    
    @staticmethod
    def parse_report(data: bytes):
        return struct.unpack('<IB9sHBB2sBB32sBB3sB88s80s32s', data)
