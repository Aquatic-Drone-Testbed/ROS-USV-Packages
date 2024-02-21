from dataclasses import dataclass, field
import struct

@dataclass(frozen=True, order=True)
class RMReport:
    field01: int
    ranges: list[int]
    fieldx_1: list[int]
    
    status: list[int]
    fieldx_2: list[int]
    warmup_time: int
    signal_strength: int
    
    fieldx_3: list[int]
    range_id: int
    fieldx_4: list[int]
    auto_gain: int
    fieldx_5: list[int]
    gain: int
    auto_sea: int
    fieldx_6: list[int]
    sea_value: int
    rain_enabled: int
    fieldx_7: list[int]
    rain_value: int
    ftc_enabled: int
    fieldx_8: list[int]
    ftc_value: int
    auto_tune: int
    fieldx_9: list[int]
    tune: int
    bearing_offset: int
    interference_rejection: int
    fieldx_10: list[int]
    target_expansion: int
    fieldx_11: list[int]
    mbs_enabled: int
    
    @staticmethod
    def parse_report(data: bytes):
        return struct.unpack('<I44s132sB3sBB7sB2sB3sIB3sBB3sBB3sBB3sBHB3sB13sB', data)
