from dataclasses import dataclass, field
import struct
import socket

@dataclass(frozen=True, order=True)
class LocationInfo:
    field1: int
    field2: int
    model_id: int
    field3: int
    field4: int
    field5: int
    field6: int
    data_ip: str
    data_port: int
    radar_ip: str
    radar_port: int
    
    @staticmethod
    def parse(data: bytes):
        fields = list(struct.unpack('<IIBBHIII4sI4s', data))
        
        fields[7] = socket.inet_ntoa(struct.pack('>I', fields[7]))
        fields[8] = struct.unpack('<2H', fields[8])[0]
        fields[9] = socket.inet_ntoa(struct.pack('>I', fields[9]))
        fields[10] = struct.unpack('<2H', fields[10])[0]
        
        return fields
