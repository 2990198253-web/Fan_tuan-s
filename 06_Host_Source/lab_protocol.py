"""Swing Lab wire contract. Fixed sizes, CRC16-CCITT, little-endian fields."""
from dataclasses import dataclass,replace
import binascii
import struct

# The order is shared with lab_config.v, never reorder it silently.
NAMES = ('accel_ms','brake_ms','wait_ms','accel_pwm','brake_pwm','kick_distance',
         'target_speed','soft_travel','pulses','flags','phase_confirm','timeout_ms',
         'return_pwm','capture_window_adc')
DEFAULTS = (40,200,200,2500,2500,140,24,350,40,129,2,15000,1000,20)
RANGES = ((10,500),(10,300),(20,2000),(500,4000),(500,4000),(20,180),
          (4,40),(80,360),(1,40),(0,255),(1,5),(1000,15000),(500,1800),(10,40))
STATES = {0:'待机',1:'起摆试验',2:'直立平衡',3:'故障',4:'急停',5:'顶部预捕获'}
ACTIONS = {0:'准备',2:'正向加速',3:'反向加速',4:'主动制动',5:'等待下一脉冲',
           6:'硬超速减速',7:'安全回中',8:'顶部减速',9:'ADC死区减速',10:'本次结束',11:'目标速度保持',
           12:'连续补能',13:'能量回收/抑制',14:'臂速包络约束',15:'短时初始激励'}
CUTS = {0:'尚未结束加速',1:'设定加速时间到',2:'单次距离上限',3:'设定限速',
        4:'软行程/20ms预测保护',5:'接近直立',6:'ADC饱和/跳变',7:'制动超时',8:'外部停止/故障中断',9:'固定硬超速',10:'初始激励结束'}
INHIBITS = {0:'无',1:'停止',2:'急停',3:'偏离直立',4:'ADC未更新',5:'试验超时',
            6:'硬行程240计数',7:'编码器异常',8:'设定脉冲完成',9:'制动超时',10:'串口心跳丢失'}
ACKS = {0:'尚无回复',1:'接受',2:'机构未停稳/正在运行/故障，拒绝修改',3:'参数越界',4:'命令或保留字错误'}
CUTS.update({11:'顶部：速度基线未建立',12:'顶部：摆杆速度过快',13:'顶部：转臂速度过快',14:'顶部：等待接管确认',15:'已接管直立'})
INHIBITS[11]='摆杆ADC持续异常'
INHIBITS[6]='转臂硬行程保护'
INHIBITS[12]='起摆计算未按时完成'

def validate(values):
    if len(values) != 14 or any(type(v) is not int for v in values):
        raise ValueError('参数必须是14个整数')
    for name, value, (low, high) in zip(NAMES, values, RANGES):
        if not low <= value <= high:
            raise ValueError(f'{name}必须在{low}～{high}之间')
    if any(values[i] % 10 for i in (0,1,2,11)):
        raise ValueError('时间必须为10ms的整数倍')
    if not values[9]&128 and (values[6]>20 or values[7]>200):
        raise ValueError('旧模式限速最大2000、软行程最大200')
    if values[9]&128 and values[9]&124:
        raise ValueError('LAB4不能混用旧策略或反向开关')
    if values[5] > values[7]:
        raise ValueError('单次距离不能大于软行程')
    if values[9]&48 == 48:
        raise ValueError('速度方向起摆与能量估算策略不能同时开启')
    if values[9]&64 and not values[9]&32:
        raise ValueError('顶部预捕获需要LAB3速度方向策略')
    return tuple(values)

def command(seq, opcode, values=DEFAULTS):
    if opcode not in (1,2,3):
        raise ValueError('不存在串口启动命令；请用板上K1')
    values = validate(values)
    body = bytes((0x61,seq & 255,opcode,0)) + struct.pack('<14H',*values) + bytes(4)
    return b'\xC3\x3C' + body + struct.pack('<H',binascii.crc_hqx(body,0xFFFF))

def position_command(seq, target_counts, speed_counts_s):
    """Move relative to the encoder position latched at BALANCE capture."""
    if type(target_counts) is not int or not -180 <= target_counts <= 180:
        raise ValueError('目标位置必须在 -180～+180 计数之间')
    if type(speed_counts_s) is not int or not 1 <= speed_counts_s <= 30:
        raise ValueError('移动速度必须在 1～30 计数/秒之间')
    body = (bytes((0x61,seq & 255,4,0)) + struct.pack('<14H',*DEFAULTS) +
            struct.pack('<hH',target_counts,speed_counts_s))
    return b'\xC3\x3C' + body + struct.pack('<H',binascii.crc_hqx(body,0xFFFF))


def outer_gain_command(seq, p_q8, d_q8):
    """Set live outer-loop gains; works only with firmware telemetry v0x62."""
    if type(p_q8) is not int or not 0 <= p_q8 <= 32:
        raise ValueError('外环 P 必须是 0～32 的整数')
    if type(d_q8) is not int or not 0 <= d_q8 <= 8191:
        raise ValueError('外环 D 必须是 0～8191 的整数')
    body = (bytes((0x61,seq & 255,5,0)) + struct.pack('<14H',*DEFAULTS) +
            struct.pack('<HH',p_q8,d_q8))
    return b'\xC3\x3C' + body + struct.pack('<H',binascii.crc_hqx(body,0xFFFF))

def outer_pid_command(seq, p_q8, i_q16, d_q8):
    """Atomically set P/I/D on telemetry-v0x63 firmware (opcode 6)."""
    if type(p_q8) is not int or not 0 <= p_q8 <= 32:
        raise ValueError('外环 P 必须是 0～32 的整数')
    if type(i_q16) is not int or not 0 <= i_q16 <= 64:
        raise ValueError('外环 I 必须是 0～64 的整数')
    if type(d_q8) is not int or not 0 <= d_q8 <= 8191:
        raise ValueError('外环 D 必须是 0～8191 的整数')
    config=list(DEFAULTS)
    config[13]=i_q16  # opcode 6 repurposes bytes 32..33; it never commits configuration.
    body=(bytes((0x61,seq & 255,6,0)) + struct.pack('<14H',*config) +
          struct.pack('<HH',p_q8,d_q8))
    return b'\xC3\x3C' + body + struct.pack('<H',binascii.crc_hqx(body,0xFFFF))

def inner_pid_command(seq, p, i_q8, d):
    """Set inner angle PID on telemetry-v0x64 and newer firmware (opcode 7)."""
    for name,value,limit in (('P',p,1000),('I',i_q8,128),('D',d,1200)):
        if type(value) is not int or not 0 <= value <= limit:
            raise ValueError(f'内环 {name} 必须是 0～{limit} 的整数')
    config=list(DEFAULTS)
    config[13]=i_q8
    body=(bytes((0x61,seq & 255,7,0)) + struct.pack('<14H',*config) +
          struct.pack('<HH',p,d))
    return b'\xC3\x3C' + body + struct.pack('<H',binascii.crc_hqx(body,0xFFFF))

@dataclass(frozen=True)
class Sample:
    seq: int
    status: int
    state: int
    action: int
    cut: int
    inhibit: int
    ack_seq: int
    ack: int
    config_safe: bool
    angle_x2: int
    encoder: int
    motor: int
    step_ms: int
    pulse_index: int
    revision: int
    config: tuple
    trial_id: int
    rx_crc_errors: int
    run_ms: int
    actual_accel_ms: int
    arm_speed: int = 0  # counts/10ms, firmware sampled, not host receive-time derivative
    protocol_version: int = 0x61
    @property
    def reference_speed_rad_s(self):
        if not self.config[9]&128 or self.state not in (1,5):return None
        v=self.actual_accel_ms-65536 if self.actual_accel_ms>=32768 else self.actual_accel_ms
        return v/256
    @property
    def zero_adc(self):
        if not self.config[9]&128 or (self.protocol_version>=0x65 and self.state==2):return None
        return self.step_ms
    @property
    def inner_target_adc(self):
        if self.protocol_version<0x65 or not self.config[9]&128 or self.state!=2:return None
        return self.step_ms/2
    @property
    def position_error_counts(self):
        if not self.config[9]&128 or self.state!=2:return None
        value=self.actual_accel_ms
        return value-65536 if value>=32768 else value
    @property
    def position_target_counts(self):
        error=self.position_error_counts
        return self.encoder+error if error is not None else None
    @property
    def zero_valid(self):return bool(self.config[9]&128 and self.pulse_index&32768)
    @property
    def zero_busy(self):return bool(self.config[9]&128 and self.pulse_index&16384)
    @property
    def zero_failed(self):return bool(self.config[9]&128 and self.pulse_index&8192)
    @property
    def energy_error_q8(self):
        if not self.config[9]&16 or self.config[9]&32:return None
        return self.actual_accel_ms-65536 if self.actual_accel_ms>=32768 else self.actual_accel_ms
    @property
    def relay_delta_x2(self):
        if not self.config[9]&32:return None
        return self.actual_accel_ms-65536 if self.actual_accel_ms>=32768 else self.actual_accel_ms

def unpack(packet):
    if len(packet)!=64 or packet[:2]!=b'\xA5\x5A' or packet[2] not in (0x61,0x62,0x63,0x64,0x65):
        raise ValueError('不是起摆试验台固件')
    if binascii.crc_hqx(packet[2:62],0xFFFF)!=struct.unpack_from('<H',packet,62)[0]:
        raise ValueError('遥测CRC错误')
    values=validate(struct.unpack_from('<14H',packet,26))
    state,angle=packet[5],struct.unpack_from('<H',packet,12)[0]
    if state not in STATES or angle>2046 or packet[10]>4:
        raise ValueError('遥测字段异常')
    return Sample(packet[3],packet[4],state,packet[6],packet[7],packet[8],packet[9],packet[10],
                  bool(packet[11]&1),angle,struct.unpack_from('<i',packet,14)[0],
                  struct.unpack_from('<h',packet,18)[0],*struct.unpack_from('<3H',packet,20),
                  values,*struct.unpack_from('<4H',packet,54),
                  (packet[11]>>1)-128 if packet[11]&128 else packet[11]>>1,packet[2])

def pack(sample):
    p=bytearray(64);p[:3]=bytes((0xA5,0x5A,sample.protocol_version))
    p[3:12]=bytes((sample.seq,sample.status,sample.state,sample.action,sample.cut,
                     sample.inhibit,sample.ack_seq,sample.ack,((sample.arm_speed&127)<<1)|int(sample.config_safe)))
    struct.pack_into('<Hih3H',p,12,sample.angle_x2,sample.encoder,sample.motor,
                     sample.step_ms,sample.pulse_index,sample.revision)
    struct.pack_into('<14H',p,26,*validate(sample.config))
    struct.pack_into('<4H',p,54,sample.trial_id,sample.rx_crc_errors,sample.run_ms,sample.actual_accel_ms)
    struct.pack_into('<H',p,62,binascii.crc_hqx(p[2:62],0xFFFF));return bytes(p)

class Decoder:
    def __init__(self):
        self.buffer=bytearray();self.crc_errors=0;self.gaps=0;self.resets=0;self.previous=None
    def feed(self,data):
        self.buffer.extend(data);result=[]
        while len(self.buffer)>=3:
            index=self.buffer.find(b'\xA5\x5A')
            if index<0:
                del self.buffer[:-2];break
            if index:del self.buffer[:index]
            if len(self.buffer)<64:break
            try:sample=unpack(bytes(self.buffer[:64]))
            except ValueError:
                self.crc_errors+=1;del self.buffer[0];continue
            del self.buffer[:64]
            if self.previous is not None:
                difference=(sample.seq-self.previous)&255
                if 1<difference<128:self.gaps+=difference-1
                elif difference==0 or difference>=128:self.resets+=1
            self.previous=sample.seq;result.append(sample)
        return result

def self_test():
    s=Sample(255,16,0,0,0,0,7,1,True,450,-123,-4000,120,1,3,DEFAULTS,8,0,220,120)
    packet=pack(s);assert unpack(packet)==s and len(command(1,1))==40
    assert command(1,3)[4]==3
    move=position_command(9,-25,12)
    assert len(move)==40 and move[4]==4 and struct.unpack_from('<hH',move,34)==(-25,12)
    assert binascii.crc_hqx(move[2:38],0xFFFF)==struct.unpack_from('<H',move,38)[0]
    gains=outer_gain_command(10,12,512)
    assert len(gains)==40 and gains[4]==5 and struct.unpack_from('<HH',gains,34)==(12,512)
    assert binascii.crc_hqx(gains[2:38],0xFFFF)==struct.unpack_from('<H',gains,38)[0]
    assert unpack(pack(replace(s,protocol_version=0x62))).protocol_version==0x62
    pid=outer_pid_command(11,12,4,512)
    assert len(pid)==40 and pid[4]==6 and struct.unpack_from('<H',pid,32)[0]==4
    assert struct.unpack_from('<HH',pid,34)==(12,512)
    assert binascii.crc_hqx(pid[2:38],0xFFFF)==struct.unpack_from('<H',pid,38)[0]
    assert unpack(pack(replace(s,protocol_version=0x63))).protocol_version==0x63
    inner=inner_pid_command(12,220,16,400)
    assert len(inner)==40 and inner[4]==7 and struct.unpack_from('<H',inner,32)[0]==16
    assert struct.unpack_from('<HH',inner,34)==(220,400)
    assert binascii.crc_hqx(inner[2:38],0xFFFF)==struct.unpack_from('<H',inner,38)[0]
    assert unpack(pack(replace(s,protocol_version=0x64))).protocol_version==0x64
    balance=replace(s,protocol_version=0x65,state=2,angle_x2=1510,step_ms=1512,
                    actual_accel_ms=5,encoder=100,pulse_index=32768)
    parsed=unpack(pack(balance))
    assert parsed.inner_target_adc==756 and parsed.zero_adc is None
    assert parsed.position_target_counts==105 and parsed.position_error_counts==5
    d=Decoder();out=[]
    for b in b'noise'+packet:out.extend(d.feed(bytes((b,))))
    assert out==[s]
    bad=bytearray(packet);bad[33]^=8
    assert d.feed(bytes(bad)+packet)==[s] and d.crc_errors==1
    for index,(lo,hi) in enumerate(RANGES):
        for bad in (lo-1,hi+1):
            vals=list(DEFAULTS);vals[index]=bad
            try:validate(vals)
            except ValueError:pass
            else:raise AssertionError((index,bad))
    for i in (0,1,2,11):
        vals=list(DEFAULTS);vals[i]+=1
        try:validate(vals)
        except ValueError:pass
        else:raise AssertionError('non-10ms duration accepted')
    try:command(1,4)
    except ValueError:pass
    else:raise AssertionError('Unexpected remote start')
    for energy in (-8192,-32,0,32,512):
        cfg=list(DEFAULTS);cfg[9]=17;cfg[6]=16;cfg[7]=180
        e=Sample(1,16,1,12,0,0,0,0,False,450,0,0,0,1,1,tuple(cfg),1,0,10,energy&65535)
        assert unpack(pack(e)).energy_error_q8==energy
        cfg[9]=33
        r=Sample(1,16,1,13,12,0,0,0,False,450,0,0,0,1,1,tuple(cfg),1,0,10,energy&65535)
        assert unpack(pack(r)).relay_delta_x2==energy and r.energy_error_q8 is None
    cfg=list(DEFAULTS);cfg[9]=49
    try:validate(cfg)
    except ValueError:pass
    else:raise AssertionError('conflicting strategies accepted')
    p=Sample(1,16,5,8,10,0,0,0,False,1450,200,1000,775,32769,1,DEFAULTS,1,0,10,768)
    assert unpack(pack(p)).state==5 and p.zero_valid and p.zero_adc==775 and p.reference_speed_rad_s==3.0
    print('SWING_LAB4_PROTOCOL_PASS relay_signed energy_signed CRC resync fragmentation ranges durations no_remote_start')
