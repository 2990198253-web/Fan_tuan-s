from pathlib import Path
import sys
from lab_protocol import Decoder,self_test
self_test()
decoder=Decoder();samples=decoder.feed(bytes(int(v,16) for v in Path(sys.argv[1]).read_text().split()))
assert len(samples)>50 and decoder.crc_errors==0
assert all(s.config[9]==129 for s in samples)
assert any(not s.zero_valid and s.state==0 for s in samples)
assert any(s.zero_busy and s.motor==0 and s.state==0 for s in samples)
assert any(s.zero_valid and s.zero_adc==775 and s.state==0 for s in samples)
assert any(s.state==1 and s.zero_valid and s.motor>0 for s in samples)
assert any(s.state==2 and s.zero_adc==775 and s.cut==15 for s in samples)
assert any(s.state==3 and s.inhibit==6 and s.motor==0 for s in samples)
assert any(s.zero_failed and not s.zero_valid and s.state==0 for s in samples)
assert all(s.motor==0 for s in samples if s.state in (0,3,4) or not s.zero_valid)
print(f'LAB4_UART_HOST_PASS frames={len(samples)} CRC=0 zero_flags calibrated775 pump handoff hard448 failed_calibration')
