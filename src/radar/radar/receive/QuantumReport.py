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
        fields = list(struct.unpack('<IB9sHBB2sBB32sBB3sB88s80s32s', data))
        fields[2] = struct.unpack('<9B', fields[2])
        fields[6] = struct.unpack('<2B', fields[6])
        fields[9] = list(struct.unpack('<8s8s8s8s', fields[9]))
        fields[12] = struct.unpack('<3B', fields[12])
        fields[14] = struct.unpack('<88B', fields[14])
        fields[15] = struct.unpack('<20I', fields[15])
        fields[16] = struct.unpack('<8I', fields[16])
        return fields
