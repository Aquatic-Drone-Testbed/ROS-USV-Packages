import struct

QuantumScanHeader = {
    'type': 'I',
    'seq_num': 'H',
    'something_1': 'H',
    'scan_len': 'H',
    'num_spokes': 'H',
    'something_3': 'H',
    'returns_per_range': 'H',
    'azimuth': 'H',
    'data_len': 'H',
}

class QuantumScanPacket:
    def __init__(self, data: bytes):
        scan_header = data[:20]
        packed_scan_data = data[20:]
        
        header = SQuantumScanDataHeader._make(struct.unpack('<IHHHHHHHH', scan_header))
        
    
        self.field01 = fields[0]
        self.ranges = fields[1:12]
        self.status = fields[44]
        self.warmup_time = fields[47]
        self.signal_strength = fields[48]
        self.range_id = fields[52]
        self.auto_gain = fields[53]
        self.gain = fields[56]
        self.auto_sea = fields[60]
        self.sea_value = fields[61]
        self.rain_enabled = fields[62]
        self.rain_value = fields[63]
        self.ftc_enabled = fields[64]
        self.ftc_value = fields[65]
        self.auto_tune = fields[66]
        self.tune = fields[68]
        self.bearing_offset = fields[69]
        self.interference_rejection = fields[70]
        self.target_expansion = fields[71]
        self.mbs_enabled = fields[85]