from dataclasses import dataclass, field
import struct

@dataclass(frozen=True, order=True)
class QuantumScan:
    type: int
    seq_num: int
    something_1: int
    scan_len: int
    num_spokes: int
    something_3: int
    returns_per_range: int
    azimuth: int
    data_len: int
    data: list[int]
    
    @staticmethod
    def parse_header(data: bytes):
        return struct.unpack('<IHHHHHHHH', data)
    
    @staticmethod
    def parse_data(data: bytes):
        unpacked_data = []
        i = 0
        while i < len(data):
            if data[i] == 0x5C:
                unpacked_data.extend([data[i+2]] * data[i+1])
                i += 3
            else:
                unpacked_data.append(data[i])
                i += 1

        return unpacked_data
