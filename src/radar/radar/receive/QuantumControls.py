from dataclasses import dataclass, field
import struct

@dataclass(frozen=True, order=True)
class QuantumControls:
    gain_auto: int
    gain: int
    color_gain_auto: int
    color_gain: int
    sea_auto: int
    sea: int
    rain_auto: int
    rain: int
    
    @staticmethod
    def parse_controls(data: bytes):
        fields = struct.unpack('<BBBBBBBB', data)
        return fields
