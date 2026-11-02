"""Cross-check the bytes actually emitted by the simulated FPGA UART."""
import sys
from pathlib import Path
from lab_protocol import Decoder,self_test

def main(path):
    self_test()
    stream=bytes(int(v,16) for v in Path(path).read_text().split())
    decoder=Decoder();samples=decoder.feed(stream)
    assert len(samples)>50 and decoder.crc_errors==0
    assert any(s.ack_seq==10 and s.ack==1 and s.revision==1 and s.config[0]==200 and s.config[6]==12 for s in samples)
    assert any(s.ack_seq==11 and s.ack==2 and s.config[0]==200 for s in samples)
    assert any(s.state==1 and s.motor>0 for s in samples)
    assert any(s.ack_seq==12 and s.state==0 and s.motor==0 for s in samples)
    assert any(s.inhibit==6 and s.state==3 and s.motor==0 for s in samples)
    assert any(s.state==2 and s.config[9]&1 for s in samples)
    assert any(s.config[9]&16 and s.state==1 and s.energy_error_q8==512 for s in samples)
    assert any(s.config[9]&32 and s.state==1 and s.relay_delta_x2==4 and s.motor==2500 for s in samples)
    assert any(s.config[9]&32 and s.state==1 and s.relay_delta_x2==-4 and s.motor==-2500 for s in samples)
    assert any(s.config[9]&32 and s.state==1 and s.cut==12 for s in samples)
    assert any(s.config[9]&32 and s.state==2 and s.cut==15 for s in samples)
    assert any(s.config[9]==97 and s.state==5 and s.action in (7,8) for s in samples)
    assert any(s.config[9]==97 and s.state==2 and s.cut==15 for s in samples)
    assert any(s.inhibit==10 and s.state==0 and s.motor==0 for s in samples)
    assert all(s.motor==0 for s in samples if s.state in (0,3,4))
    assert all(s.config[0]!=300 for s in samples)
    print(f'SWING_LAB_RTL_HOST_CROSSCHECK_PASS frames={len(samples)} CRC=0 config_speed busy_reject stop travel balance heartbeat')

if __name__=='__main__':main(sys.argv[1])
