"""Dedicated, explicit-apply Swing Lab. Native Windows COM + Tk; no Qt/pip."""
import argparse
from collections import deque
import csv
import ctypes
from dataclasses import replace
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from ctypes import wintypes
from lab_serial import Serial, ports
from lab_protocol import (NAMES,DEFAULTS,RANGES,STATES,ACTIONS,CUTS,INHIBITS,ACKS,
                          Sample,Decoder,command,position_command,outer_gain_command,outer_pid_command,inner_pid_command,
                          pack,validate,self_test)

LABELS=('驱动/初踢时间(ms)','制动最长时间(ms)','旧脉冲等待(ms)','补能最大力度(%)',
        '制动力度(%)','初踢距离(计数)','目标/臂速上限(计数/s)','软行程(计数)','脉冲/补能段数',
        '模式','相位确认次数','试验超时(ms)','回中力度(%)','接管窗口(ADC)')
SCALES={3:50,4:50,6:0.01,12:50}

def output_root():
    root=Path(sys.executable).parent if getattr(sys,'frozen',False) else Path(__file__).resolve().parent
    try:(root/'swing_lab_logs').mkdir(exist_ok=True)
    except OSError:
        root=Path(os.environ.get('LOCALAPPDATA',str(Path.home())))/'SwingLab'
        (root/'swing_lab_logs').mkdir(parents=True,exist_ok=True)
    return root

class Lab:
    def __init__(self,root,demo=False,log_root=None,demo_version=0x65):
        self.root=root;self.demo=demo;self.demo_version=demo_version;self.serial=None;self.connected=False
        self.decoder=Decoder();self.latest=None;self.last_rx=0;self.pending=None;self.target_pending=None;self.gain_pending=None;self.inner_pending=None
        self.commanded_target=0;self.last_balance_trial=None;self.move_ready_at=0
        self.sine_running=False;self.sine_return_target=None;self.sine_trial_id=None
        self.sine_center=0;self.sine_amplitude=0;self.sine_period=0;self.sine_speed=0
        self.sine_started_at=0;self.sine_next_at=0
        self.confirmed_gain=None;self.confirmed_inner=None;self.last_protocol_version=None
        self.seq=0;self.last_query=0;self.initial_read=False;self.rows=deque(maxlen=3000)
        self.chart_rows=deque(maxlen=6000);self.chart_marks=deque(maxlen=100);self.last_zero_adc=None
        self.large_chart_window=None;self.large_chart_canvas=None
        self.log_root=Path(log_root) if log_root else output_root()
        self.log=None;self.writer=None;self.event_stream=None;self.base=None
        self.count=0;self.confirmed_applies=0;self.demo_seq=0
        self.demo_cfg=DEFAULTS;self.demo_revision=0;self.demo_ack_seq=0;self.demo_ack=0
        self.demo_safe=True;self.started=time.monotonic();self.last_flush=0;self.last_draw=0
        self.loading=False;self.editor_dirty=False;self.last_active_cfg=None
        root.title('J280 LAB4 R14：双环 PID 与正弦横杆轨迹')
        root.geometry('1220x1030');root.minsize(1040,850)
        toolbar=ttk.Frame(root,padding=8);toolbar.pack(fill='x')
        ttk.Label(toolbar,text='串口').pack(side='left')
        self.port=tk.StringVar(value='COM3');self.portbox=ttk.Combobox(toolbar,textvariable=self.port,width=12)
        self.portbox.pack(side='left',padx=5)
        ttk.Button(toolbar,text='刷新',command=self.refresh_ports).pack(side='left')
        self.connect_button=ttk.Button(toolbar,text='连接并记录',command=self.connect_toggle);self.connect_button.pack(side='left',padx=6)
        ttk.Button(toolbar,text='串口停止（K2仍有效）',command=self.stop).pack(side='left',padx=6)
        self.read_button=ttk.Button(toolbar,text='用板上参数填入',command=self.read_board);self.read_button.pack(side='left',padx=6)
        ttk.Label(toolbar,text='先扶直按K3标定，放下后K1；K4急停').pack(side='right')
        editor=ttk.LabelFrame(root,text='编辑区：改数值不会直接生效，必须点击下发并等回读确认',padding=8);editor.pack(fill='x',padx=8)
        self.fields={};self.spins={};self.modecontrols={};self.flagvars=[tk.IntVar(value=0) for _ in range(8)]
        for b,var in enumerate(self.flagvars):var.set((DEFAULTS[9]>>b)&1)
        slots=[i for i in range(14) if i!=9]
        for n,i in enumerate(slots):
            r,c=divmod(n,4);c*=2
            ttk.Label(editor,text=LABELS[i]).grid(row=r,column=c,sticky='w',padx=(4,2),pady=4)
            scale=SCALES.get(i,1);value=DEFAULTS[i]/scale
            var=tk.StringVar(value=f'{value:g}');self.fields[i]=var
            lo,hi=RANGES[i];step=100 if i==6 else 5 if i in (3,4,12) else 10 if i in (0,1,2,11) else 1
            spin=ttk.Spinbox(editor,textvariable=var,from_=lo/scale,to=hi/scale,increment=step,width=8)
            spin.grid(row=r,column=c+1,padx=(0,14));self.spins[i]=spin
        modebar=ttk.Frame(editor);modebar.grid(row=4,column=0,columnspan=8,sticky='w',pady=6)
        for b,text in ((1,'旧脉冲跟随摆杆相位'),(0,'接近直立自动切平衡'),(6,'旧LAB3预捕获'),(2,'起摆方向反向（停机后修改）')):
            control=ttk.Checkbutton(modebar,text=text,variable=self.flagvars[b]);control.pack(side='left',padx=8);self.modecontrols[b]=control
        buttons=ttk.Frame(editor);buttons.grid(row=5,column=0,columnspan=8,sticky='w')
        self.apply_button=ttk.Button(buttons,text='下发参数并回读确认',command=self.apply);self.apply_button.pack(side='left',padx=4)
        ttk.Button(buttons,text='单次甩动预设',command=lambda:self.preset(False)).pack(side='left',padx=4)
        ttk.Button(buttons,text='12次往复预设',command=lambda:self.preset(True)).pack(side='left',padx=4)
        ttk.Button(buttons,text='保存参数方案',command=self.save_profile).pack(side='left',padx=4)
        ttk.Button(buttons,text='载入方案（不自动下发）',command=self.load_profile).pack(side='left',padx=4)
        strategybar=ttk.Frame(editor);strategybar.grid(row=6,column=0,columnspan=8,sticky='w',pady=4)
        self.strategy=tk.StringVar(value='LAB4 参考能量起摆')
        ttk.Label(strategybar,text='起摆策略').pack(side='left')
        self.strategybox=ttk.Combobox(strategybar,textvariable=self.strategy,state='readonly',width=28,
            values=('LAB4 参考能量起摆','LAB3 摆杆速度方向起摆','LAB2 能量估算','LAB1 速度脉冲','LAB1 PWM脉冲'))
        self.strategybox.pack(side='left',padx=6);self.strategybox.bind('<<ComboboxSelected>>',self.choose_strategy)
        ttk.Button(strategybar,text='LAB4 起摆预设',command=self.relay_preset).pack(side='left',padx=8)
        ttk.Button(strategybar,text='连续能量起摆预设',command=self.energy_preset).pack(side='left',padx=8)
        move=ttk.LabelFrame(root,text='直立后移动横杆：目标相对接管位置（编码器计数）',padding=8)
        move.pack(fill='x',padx=8,pady=(5,0))
        ttk.Label(move,text='目标 -180～+180').pack(side='left')
        self.target_var=tk.StringVar(value='0')
        ttk.Entry(move,textvariable=self.target_var,width=8).pack(side='left',padx=5)
        ttk.Label(move,text='最大速度 1～30 计数/秒').pack(side='left',padx=(15,0))
        self.speed_var=tk.StringVar(value='10')
        ttk.Spinbox(move,textvariable=self.speed_var,from_=1,to=30,width=5).pack(side='left',padx=5)
        ttk.Label(move,text='单次最大位移 1～180 计数').pack(side='left',padx=(10,0))
        self.move_step_var=tk.StringVar(value='60')
        ttk.Spinbox(move,textvariable=self.move_step_var,from_=1,to=180,width=5).pack(side='left',padx=5)
        self.target_buttons=[]
        for label,callback in (('发送目标',self.apply_position),('左移一步',lambda:self.move_by(-1)),
                               ('右移一步',lambda:self.move_by(1)),('回接管点',self.move_home_step)):
            button=ttk.Button(move,text=label,command=callback);button.pack(side='left',padx=4)
            self.target_buttons.append(button)
        self.move_text=tk.StringVar(value='R14 位流：横杆硬行程停机已关闭；位置目标范围仍为 -180～+180，K2/K4 与摆角停机保留。')
        ttk.Label(root,textvariable=self.move_text,wraplength=1180,padding=(8,2)).pack(fill='x')
        sinebar=ttk.LabelFrame(root,text='直立平衡后：横杆按正弦位置目标往复',padding=8)
        sinebar.pack(fill='x',padx=8,pady=(5,0))
        ttk.Label(sinebar,text='幅度 1～180 计数').pack(side='left')
        self.sine_amplitude_var=tk.StringVar(value='120')
        ttk.Spinbox(sinebar,textvariable=self.sine_amplitude_var,from_=1,to=180,width=5).pack(side='left',padx=5)
        ttk.Label(sinebar,text='周期 4～120 秒').pack(side='left',padx=(12,0))
        self.sine_period_var=tk.StringVar(value='32')
        ttk.Spinbox(sinebar,textvariable=self.sine_period_var,from_=4,to=120,width=5).pack(side='left',padx=5)
        ttk.Label(sinebar,text='速度上限 1～30 计数/秒').pack(side='left',padx=(12,0))
        self.sine_speed_var=tk.StringVar(value='30')
        ttk.Spinbox(sinebar,textvariable=self.sine_speed_var,from_=1,to=30,width=5).pack(side='left',padx=5)
        self.sine_start_button=ttk.Button(sinebar,text='开始正弦往复',command=self.start_sine)
        self.sine_start_button.pack(side='left',padx=(14,4))
        self.sine_stop_button=ttk.Button(sinebar,text='停止并回中心',command=self.stop_sine)
        self.sine_stop_button.pack(side='left',padx=4)
        self.sine_text=tk.StringVar(value='目标围绕启动时的横杆位置往复；曲线显示板端轨迹目标与实际位置。')
        ttk.Label(root,textvariable=self.sine_text,wraplength=1180,padding=(8,2)).pack(fill='x')
        tune=ttk.LabelFrame(root,text='位置外环实时调节（P：位置误差；I：累计误差；D：横杆速度阻尼）',padding=8)
        tune.pack(fill='x',padx=8,pady=(5,0))
        ttk.Label(tune,text='外环 P  0～32').pack(side='left')
        self.outer_p_var=tk.StringVar(value='15')
        ttk.Spinbox(tune,textvariable=self.outer_p_var,from_=0,to=32,width=7).pack(side='left',padx=5)
        ttk.Label(tune,text='外环 I  0～64').pack(side='left',padx=(18,0))
        self.outer_i_var=tk.StringVar(value='19')
        self.outer_i_spin=ttk.Spinbox(tune,textvariable=self.outer_i_var,from_=0,to=64,width=7)
        self.outer_i_spin.pack(side='left',padx=5)
        ttk.Label(tune,text='外环 D  0～8191').pack(side='left',padx=(18,0))
        self.outer_d_var=tk.StringVar(value='5000')
        ttk.Spinbox(tune,textvariable=self.outer_d_var,from_=0,to=8191,width=8).pack(side='left',padx=5)
        self.gain_apply_button=ttk.Button(tune,text='发送外环 P/I/D',command=self.apply_outer_gains)
        self.gain_apply_button.pack(side='left',padx=9)
        self.gain_text=tk.StringVar(value='编辑值尚未发送。R6/R5/R4 支持外环 P/I/D；R3 仅支持 P/D。')
        ttk.Label(root,textvariable=self.gain_text,wraplength=1180,padding=(8,2)).pack(fill='x')
        inner=ttk.LabelFrame(root,text='摆杆内环实时调节（P：倾斜纠正；I：长期误差；D：角速度阻尼）',padding=8)
        inner.pack(fill='x',padx=8,pady=(5,0))
        ttk.Label(inner,text='内环 P  0～1000').pack(side='left')
        self.inner_p_var=tk.StringVar(value='115')
        ttk.Spinbox(inner,textvariable=self.inner_p_var,from_=0,to=1000,width=7).pack(side='left',padx=5)
        ttk.Label(inner,text='内环 I  0～0.5（小数）').pack(side='left',padx=(18,0))
        self.inner_i_var=tk.StringVar(value='0.25')
        ttk.Spinbox(inner,textvariable=self.inner_i_var,from_=0,to=0.5,increment=1/256,width=9).pack(side='left',padx=5)
        ttk.Label(inner,text='内环 D  0～1200').pack(side='left',padx=(18,0))
        self.inner_d_var=tk.StringVar(value='520')
        ttk.Spinbox(inner,textvariable=self.inner_d_var,from_=0,to=1200,width=8).pack(side='left',padx=5)
        self.inner_apply_button=ttk.Button(inner,text='发送内环 P/I/D',command=self.apply_inner_gains)
        self.inner_apply_button.pack(side='left',padx=9)
        self.inner_text=tk.StringVar(value='内环编辑值尚未发送；请以本次板端 ACK 为生效依据。')
        ttk.Label(root,textvariable=self.inner_text,wraplength=1180,padding=(8,2)).pack(fill='x')
        livebar=ttk.Frame(root,padding=(8,1));livebar.pack(fill='x')
        self.auto_tune_enabled=tk.BooleanVar(value=True)
        ttk.Checkbutton(livebar,text='PID 改值后自动在线下发（约 250 ms 停顿，等待板端 ACK）',
                        variable=self.auto_tune_enabled,command=self.auto_tune_toggled).pack(side='left')
        ttk.Label(livebar,text='运行中可调；连续输入只发送最后一组有效值。').pack(side='left',padx=12)
        self.auto_dirty={'outer':False,'inner':False}
        self.auto_ready_at={'outer':0.0,'inner':0.0}
        for var in (self.outer_p_var,self.outer_i_var,self.outer_d_var):
            var.trace_add('write',lambda *_:self.tune_edited('outer'))
        for var in (self.inner_p_var,self.inner_i_var,self.inner_d_var):
            var.trace_add('write',lambda *_:self.tune_edited('inner'))
        root.bind('<Control-Left>',lambda event:self.move_by(-1))
        root.bind('<Control-Right>',lambda event:self.move_by(1))
        root.bind('<Control-Home>',lambda event:self.move_home_step())
        self.apply_text=tk.StringVar(value='未连接。先连接，等待板上参数和停稳状态。')
        for var in (*self.fields.values(),*self.flagvars):var.trace_add('write',self.edited)
        self.refresh_field_states()
        ttk.Label(root,textvariable=self.apply_text,wraplength=1180,padding=8).pack(fill='x')
        self.board_text=tk.StringVar(value='板上实际参数：尚未收到 LAB3 回读')
        ttk.Label(root,textvariable=self.board_text,wraplength=1180,padding=(8,0)).pack(fill='x')
        self.live_text=tk.StringVar(value='无数据；请先连接串口并下载配套位流。')
        ttk.Label(root,textvariable=self.live_text,wraplength=1180,padding=8,font=('Microsoft YaHei UI',11)).pack(fill='x')
        chartbar=ttk.Frame(root,padding=(8,2));chartbar.pack(fill='x')
        ttk.Label(chartbar,text='平衡波形：青色＝实际，黄色＝目标；停止后保留最近一次平衡').pack(side='left')
        ttk.Label(chartbar,text='时间窗').pack(side='left',padx=(20,4))
        self.window_var=tk.StringVar(value='20')
        ttk.Combobox(chartbar,textvariable=self.window_var,values=('5','10','20','30','60'),state='readonly',width=5).pack(side='left')
        ttk.Button(chartbar,text='放大曲线（独立窗口）',command=self.open_large_chart).pack(side='right')
        self.canvas=tk.Canvas(root,bg='#111827',highlightthickness=0);self.canvas.pack(fill='both',expand=True,padx=8,pady=5)
        self.note=tk.StringVar(value='设定时长不保证全部执行：距离、行程、硬超速、ADC异常可提前中断，并明确报告原因。')
        ttk.Label(root,textvariable=self.note,wraplength=1180,padding=8).pack(fill='x')
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.refresh_ports();self.buttons();root.after(20,self.poll)

    def refresh_ports(self):
        try:self.portbox['values']=ports()
        except OSError:self.portbox['values']=()
    def values(self):
        vals=list(DEFAULTS)
        for i,var in self.fields.items():
            number=float(var.get())*SCALES.get(i,1)
            if not math.isfinite(number) or abs(number-round(number))>1e-6:raise ValueError(f'{LABELS[i]}精度不合法')
            vals[i]=round(number)
        vals[9]=sum(var.get()<<b for b,var in enumerate(self.flagvars));return validate(vals)
    def fill(self,values):
        self.loading=True
        try:
            for i,var in self.fields.items():var.set(f'{values[i]/SCALES.get(i,1):g}')
            for b,var in enumerate(self.flagvars):var.set((values[9]>>b)&1)
            self.strategy.set(self.strategybox['values'][0 if values[9]&128 else 1 if values[9]&32 else 2 if values[9]&16 else 3 if values[9]&8 else 4])
        finally:self.loading=False
        self.editor_dirty=False
    def edited(self,*unused):
        self.refresh_field_states()
        if not self.loading:
            self.editor_dirty=True
            self.apply_text.set('编辑区已修改，尚未下发；板上仍使用回读参数。')
    def refresh_field_states(self):
        reference=bool(self.flagvars[7].get())
        for i,spin in self.spins.items():spin.configure(state='disabled' if reference and i in (0,1,2,4,5,8,10,12) else 'normal')
        for bit,control in self.modecontrols.items():control.configure(state='disabled' if reference and bit in (1,2,6) else 'normal')
    def preset(self,repeated):
        self.fields[6].set(f'{min(float(self.fields[6].get()),2000):g}');self.fields[7].set(f'{min(float(self.fields[7].get()),200):g}')
        self.fields[8].set('12' if repeated else '1');self.flagvars[0].set(0);self.flagvars[1].set(0)
        self.flagvars[4].set(0);self.flagvars[5].set(0);self.flagvars[6].set(0);self.flagvars[7].set(0);self.flagvars[3].set(1);self.strategy.set('LAB1 速度脉冲')
        self.apply_text.set('预设仅改编辑区，尚未下发。建议一次只改加速时间或目标速度中的一个。')
    def choose_strategy(self,*unused):
        index=self.strategybox.current()
        for bit in (3,4,5,6,7):self.flagvars[bit].set(0)
        if index==0:self.flagvars[2].set(0)
        if 0<=index<4:self.flagvars[(7,5,4,3)[index]].set(1)
        if index!=0:
            self.fields[6].set(f'{min(float(self.fields[6].get()),2000):g}')
            self.fields[7].set(f'{min(float(self.fields[7].get()),200):g}')
    def relay_preset(self):
        self.fill(DEFAULTS);self.editor_dirty=True
        self.apply_text.set('LAB4预设仅编辑：50%力度、臂速2400、软行程350；下发确认后扶直按K3标定，再放下按K1。')
    def energy_preset(self):
        cfg=list(DEFAULTS);cfg[9]=17;cfg[6]=16;cfg[7]=180;self.fill(cfg);self.editor_dirty=True
        self.apply_text.set('连续补能预设仅填入编辑区：初踢40ms、臂速上限1600、力度50%、自动平衡开启。需下发确认后K1。')
    def read_board(self):
        if self.latest and self.fresh():self.fill(self.latest.config);self.apply_text.set('已用板上实际参数填入编辑区；没有发送配置。')
    def save_profile(self):
        try:vals=self.values()
        except ValueError as e:messagebox.showerror('参数',str(e));return
        name=filedialog.asksaveasfilename(defaultextension='.json',filetypes=[('JSON','*.json')])
        if name:Path(name).write_text(json.dumps(dict(zip(NAMES,vals)),ensure_ascii=False,indent=2),encoding='utf-8')
    def load_profile(self):
        name=filedialog.askopenfilename(filetypes=[('JSON','*.json')])
        if not name:return
        try:
            data=json.loads(Path(name).read_text(encoding='utf-8-sig'));self.fill(validate(tuple(data[k] for k in NAMES)))
            self.editor_dirty=True;self.apply_text.set('已载入编辑区，尚未下发，不会自动启动。')
        except Exception as e:messagebox.showerror('参数方案',str(e))
    def event(self,kind,**data):
        if kind in ('outer_gain_confirmed','inner_gain_confirmed') and self.latest is not None and self.latest.state==2:
            title='外环' if kind=='outer_gain_confirmed' else '内环'
            self.chart_marks.append((time.monotonic()-self.started,title,data['p_q8'] if title=='外环' else data['p'],
                                     data['i_q16'] if title=='外环' else data['i_q8'],data['d_q8'] if title=='外环' else data['d']))
        if self.event_stream:
            self.event_stream.write(json.dumps(dict(time=time.monotonic()-self.started,event=kind,**data),ensure_ascii=False)+'\n');self.event_stream.flush()
    def connect_toggle(self):
        if self.connected:self.disconnect();return
        try:
            if not self.demo:self.serial=Serial(self.port.get())
            self.connected=True;self.decoder=Decoder();self.latest=None;self.initial_read=False;self.pending=None
            self.target_pending=None;self.gain_pending=None;self.inner_pending=None;self.commanded_target=0;self.last_balance_trial=None;self.move_ready_at=0
            self.confirmed_gain=None;self.confirmed_inner=None;self.last_protocol_version=None
            self.last_rx=0;self.started=time.monotonic();self.rows.clear();self.chart_rows.clear();self.chart_marks.clear();self.last_zero_adc=None;self.count=0
            folder=self.log_root/'swing_lab_logs';folder.mkdir(parents=True,exist_ok=True)
            self.base=folder/(('DEMO_' if self.demo else '')+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            self.log=self.base.with_suffix('.csv').open('w',encoding='utf-8-sig',newline='')
            self.writer=csv.writer(self.log);self.writer.writerow(('host_rx_seconds','sequence','state','action','cut_reason','inhibit_reason','angle_adc','encoder_counts','motor_command','speed_counts_per_s','step_ms','actual_accel_ms','pulse_index','trial_id','config_revision','ack_sequence','ack_status','config_safe','host_crc_errors','sequence_gaps','rx_crc_errors','energy_error_q8','relay_delta_x2','capture_diag','zero_adc','zero_valid','zero_busy','zero_failed','pole_speed_rad_s','position_error_counts','ramped_target_absolute_counts','last_requested_relative_counts','protocol_version','ack_confirmed_outer_p_q8','ack_confirmed_outer_i_q16','ack_confirmed_outer_d_q8','ack_confirmed_inner_p','ack_confirmed_inner_i_q8','ack_confirmed_inner_d','inner_angle_target_adc','outer_position_target_counts',*NAMES))
            self.event_stream=self.base.with_suffix('.jsonl').open('w',encoding='utf-8')
            source_root=Path(sys.executable).parent.parent if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent
            sources={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source_root/'src').glob('*') if p.suffix in ('.v','.vh')}
            self.base.with_suffix('.json').write_text(json.dumps(dict(protocol='LAB4',demo=self.demo,port=self.port.get(),rtl_sha256=sources,note='电脑源文件指纹不等于已下载位流；起摆配置以每帧板上回读为准；实时内外环P/I/D仅有ACK确认，没有独立回读。'),ensure_ascii=False,indent=2),encoding='utf-8')
            self.connect_button.configure(text='断开并保存');self.last_query=0
            self.apply_text.set('等待 LAB4 数据和回读，未自动下发参数。')
            self.event('connected',demo=self.demo)
        except Exception as e:
            self.disconnect();messagebox.showerror('连接失败',str(e))
    def fresh(self):return self.connected and self.latest is not None and time.monotonic()-self.last_rx<0.5
    def tune_edited(self,kind):
        if self.auto_tune_enabled.get():
            self.auto_dirty[kind]=True
            self.auto_ready_at[kind]=time.monotonic()+0.25
    def auto_tune_toggled(self):
        if self.auto_tune_enabled.get():
            for kind in self.auto_dirty:self.tune_edited(kind)
        else:
            for kind in self.auto_dirty:self.auto_dirty[kind]=False
    def auto_tune(self,now):
        if not self.auto_tune_enabled.get() or not self.fresh() or self.latest.state not in (0,2):return
        if self.pending or self.target_pending or self.gain_pending or self.inner_pending:return
        version=self.latest.protocol_version
        for kind in ('outer','inner'):
            if not self.auto_dirty[kind] or now<self.auto_ready_at[kind]:continue
            if kind=='outer' and version not in (0x62,0x63,0x64,0x65):continue
            if kind=='inner' and version not in (0x64,0x65):continue
            # A delayed edit following a manual send must not resend the same
            # values: board-side gain updates clear the corresponding I state.
            try:
                desired=(int(self.outer_p_var.get()),
                         int(self.outer_i_var.get()) if version!=0x62 else None,
                         int(self.outer_d_var.get())) if kind=='outer' else (
                         int(self.inner_p_var.get()),round(float(self.inner_i_var.get())*256),
                         int(self.inner_d_var.get()))
            except ValueError:
                self.auto_dirty[kind]=False
                continue
            if desired==(self.confirmed_gain if kind=='outer' else self.confirmed_inner):
                self.auto_dirty[kind]=False
                continue
            self.auto_dirty[kind]=False
            if kind=='outer':self.apply_outer_gains()
            else:self.apply_inner_gains()
            break
    def buttons(self):
        ready=self.fresh() and self.latest.config_safe and self.pending is None and self.target_pending is None and self.gain_pending is None and self.inner_pending is None
        self.apply_button.configure(state='normal' if ready else 'disabled')
        self.read_button.configure(state='normal' if self.fresh() else 'disabled')
        target_ready=(self.fresh() and self.latest.state==2 and not self.sine_running
                      and self.sine_return_target is None and time.monotonic()>=self.move_ready_at
                      and self.pending is None and self.target_pending is None and self.gain_pending is None and self.inner_pending is None)
        for button in self.target_buttons:button.configure(state='normal' if target_ready else 'disabled')
        self.sine_start_button.configure(state='normal' if target_ready else 'disabled')
        self.sine_stop_button.configure(state='normal' if self.sine_running else 'disabled')
        version=self.latest.protocol_version if self.fresh() else None
        self.outer_i_spin.configure(state='normal' if version in (0x63,0x64,0x65) else 'disabled')
        self.gain_apply_button.configure(text='发送外环 P/I/D' if version in (0x63,0x64,0x65) else '发送外环 P/D')
        gain_ready=(self.fresh() and version in (0x62,0x63,0x64,0x65) and self.latest.state in (0,2)
                    and self.pending is None and self.target_pending is None and self.gain_pending is None and self.inner_pending is None)
        self.gain_apply_button.configure(state='normal' if gain_ready else 'disabled')
        self.inner_apply_button.configure(state='normal' if gain_ready and version in (0x64,0x65) else 'disabled')
    def send(self,opcode,values=DEFAULTS):
        if not self.connected:raise OSError('尚未连接')
        self.seq=(self.seq+1)&255;packet=command(self.seq,opcode,values)
        if self.demo:
            self.demo_ack_seq=self.seq
            if opcode==1:
                self.demo_ack=1 if self.demo_safe else 2
                if self.demo_safe:self.demo_cfg=values;self.demo_revision=(self.demo_revision+1)&65535
            else:self.demo_ack=1
        else:self.serial.write(packet)
        return self.seq
    def apply(self):
        if not self.fresh() or not self.latest.config_safe or self.pending or self.target_pending or self.gain_pending or self.inner_pending:
            self.apply_text.set('拒绝下发：需实时连接且板上报告已停稳、无故障。');return
        try:
            vals=self.values();revision=(self.latest.revision+1)&65535
            seq=self.send(1,vals);self.pending=(seq,revision,vals,time.monotonic())
            self.event('config_requested',sequence=seq,expected_revision=revision,parameters=dict(zip(NAMES,vals)))
            self.apply_text.set('已发送，尚未确认生效。等待同序号ACK、版本递增和14项参数回读一致。')
        except Exception as e:self.apply_text.set('未确认下发：'+str(e))
        self.buttons()
    def start_sine(self):
        if (not self.fresh() or self.latest.state!=2 or self.sine_running or self.sine_return_target is not None
                or self.pending or self.target_pending or self.gain_pending or self.inner_pending
                or time.monotonic()<self.move_ready_at):
            self.sine_text.set('正弦未启动：需已进入直立平衡、上一段移动完成，且没有待确认命令。');return
        try:
            amplitude=int(self.sine_amplitude_var.get())
            period=float(self.sine_period_var.get())
            speed=int(self.sine_speed_var.get())
            if not 1<=amplitude<=180:raise ValueError('幅度须在 1～180 计数。')
            if not math.isfinite(period) or not 4<=period<=120:raise ValueError('周期须在 4～120 秒。')
            if not 1<=speed<=30:raise ValueError('速度上限须在 1～30 计数/秒。')
            center=self.commanded_target
            if abs(center)+amplitude>180:raise ValueError('当前位置加幅度超出 ±180 目标范围，请先回接管点或减小幅度。')
            peak_speed=2*math.pi*amplitude/period
            if peak_speed>speed*0.8:
                raise ValueError(f'所需峰值速度 {peak_speed:.1f} 计数/秒过快；请增大周期、减小幅度或提高速度上限。')
        except ValueError as e:self.sine_text.set('正弦未启动：'+str(e));return
        now=time.monotonic()
        self.sine_center=center;self.sine_amplitude=amplitude;self.sine_period=period;self.sine_speed=speed
        self.sine_trial_id=self.latest.trial_id;self.sine_started_at=now;self.sine_next_at=now
        self.sine_running=True;self.move_ready_at=0
        self.sine_text.set(f'正弦往复运行中：中心 {center:+d}、幅度 ±{amplitude} 计数、周期 {period:g} 秒；目标约每 250 ms 更新。')
        self.event('sine_started',center=center,amplitude=amplitude,period_s=period,speed_limit=speed)
        self.buttons()
    def stop_sine(self):
        if not self.sine_running:return
        self.sine_running=False
        self.sine_return_target=self.sine_center
        self.sine_text.set(f'正弦已停止；等待当前指令确认后回到中心 {self.sine_center:+d}。')
        self.event('sine_stopped',return_target=self.sine_center)
        self.buttons()
    def update_sine(self,now):
        if not self.sine_running and self.sine_return_target is None:return
        if not self.fresh() or self.latest.state!=2 or self.latest.trial_id!=self.sine_trial_id:
            self.sine_running=False;self.sine_return_target=None
            self.sine_text.set('已退出直立平衡或通信中断，正弦目标停止发送。')
            return
        if self.pending or self.target_pending or self.gain_pending or self.inner_pending:return
        if self.sine_return_target is not None:
            target=self.sine_return_target
            if target==self.commanded_target:
                self.sine_return_target=None;self.sine_text.set('正弦已停止，目标回到中心。');return
            self.send_position(target,self.sine_speed,'sine_return')
            return
        if now<self.sine_next_at:return
        self.sine_next_at=now+0.25
        target=self.sine_center+round(self.sine_amplitude*math.sin(2*math.pi*(now-self.sine_started_at)/self.sine_period))
        if target!=self.commanded_target:self.send_position(target,self.sine_speed,'sine')
    def move_to(self,target):
        self.target_var.set(str(target));self.apply_position()
    def move_limit(self):
        limit=int(self.move_step_var.get())
        if not 1<=limit<=180:raise ValueError('单次最大位移须为 1～180 计数。')
        return limit
    def move_by(self,direction):
        try:delta=direction*self.move_limit()
        except ValueError as e:self.move_text.set(str(e));return
        target=max(-180,min(180,self.commanded_target+delta))
        if target==self.commanded_target:
            self.move_text.set('已达到 ±180 计数目标范围；未发送。');return
        self.move_to(target)
    def move_home_step(self):
        try:limit=self.move_limit()
        except ValueError as e:self.move_text.set(str(e));return
        target=self.commanded_target
        if target==0:
            self.move_text.set('目标已在接管点 0 计数；无需发送。');return
        self.move_to(target-min(limit,target) if target>0 else target+min(limit,-target))
    def apply_position(self):
        try:
            target=int(self.target_var.get());speed=int(self.speed_var.get())
            self.send_position(target,speed,'manual')
        except Exception as e:self.move_text.set('目标未发送：'+str(e))
    def send_position(self,target,speed,source):
        if (not self.fresh() or self.latest.state!=2 or self.pending or self.target_pending
                or self.gain_pending or self.inner_pending
                or (source=='manual' and (self.sine_running or self.sine_return_target is not None))):
            if source=='manual':self.move_text.set('目标未发送：需直立平衡、正弦已停止，且上一条命令已确认。')
            return False
        if source=='manual' and time.monotonic()<self.move_ready_at:
            self.move_text.set(f'上一段目标速度曲线预计还需 {self.move_ready_at-time.monotonic():.1f} 秒；暂不发送下一步。')
            return False
        try:
            if source=='manual' and abs(target-self.commanded_target)>self.move_limit():
                raise ValueError(f'本次目标距已确认目标 {abs(target-self.commanded_target)} 计数，超过单次上限 {self.move_limit()}。')
            self.seq=(self.seq+1)&255
            packet=position_command(self.seq,target,speed)
            if self.demo:self.demo_ack_seq=self.seq;self.demo_ack=1
            else:self.serial.write(packet)
            self.target_pending=(self.seq,target,speed,time.monotonic(),source)
            if source=='manual':
                self.move_text.set(f'目标 {target:+d} 计数、上限 {speed} 计数/秒已发送，等待板端 ACK；未确认前不再发送。')
            else:self.sine_text.set(f'正弦位置目标 {target:+d} 计数已发送，等待板端 ACK。')
            self.event('position_requested',sequence=self.seq,target=target,speed=speed,source=source)
        except Exception as e:
            if source=='manual':self.move_text.set('目标未发送：'+str(e))
            else:
                self.sine_running=False;self.sine_return_target=None
                self.sine_text.set('正弦目标未发送，已停止：'+str(e))
            return False
        self.buttons()
        return True
    def apply_outer_gains(self):
        if (not self.fresh() or self.latest.protocol_version not in (0x62,0x63,0x64,0x65) or self.latest.state not in (0,2)
                or self.pending or self.target_pending or self.gain_pending or self.inner_pending):
            self.gain_text.set('未发送：需要支持实时调参的固件、待机或直立平衡状态，且上一条命令已确认。')
            return
        try:
            p=int(self.outer_p_var.get());d=int(self.outer_d_var.get())
            version=self.latest.protocol_version
            i=int(self.outer_i_var.get()) if version in (0x63,0x64,0x65) else None
            self.seq=(self.seq+1)&255
            packet=outer_pid_command(self.seq,p,i,d) if version in (0x63,0x64,0x65) else outer_gain_command(self.seq,p,d)
            if self.demo:self.demo_ack_seq=self.seq;self.demo_ack=1
            else:self.serial.write(packet)
            self.gain_pending=(self.seq,p,i,d,version,time.monotonic())
            detail=f'P={p}、I={i}、D={d}' if i is not None else f'P={p}、D={d}（R3 无 I）'
            self.gain_text.set(f'外环 {detail} 已发送，等待同序号板端 ACK；编辑值尚不能视为生效。')
            self.event('outer_gain_requested',sequence=self.seq,p_q8=p,i_q16=i,d_q8=d)
        except Exception as e:self.gain_text.set('外环 PID 未发送：'+str(e))
        self.buttons()
    def apply_inner_gains(self):
        if (not self.fresh() or self.latest.protocol_version not in (0x64,0x65) or self.latest.state not in (0,2)
                or self.pending or self.target_pending or self.gain_pending or self.inner_pending):
            self.inner_text.set('未发送：需要 R5/R6 位流、待机或直立平衡状态，且上一条命令已确认。')
            return
        try:
            p=int(self.inner_p_var.get());i_float=float(self.inner_i_var.get());d=int(self.inner_d_var.get())
            if not math.isfinite(i_float) or not 0<=i_float<=0.5:
                raise ValueError('内环 I 必须在 0～0.5 之间')
            i=round(i_float*256)
            self.seq=(self.seq+1)&255
            packet=inner_pid_command(self.seq,p,i,d)
            if self.demo:self.demo_ack_seq=self.seq;self.demo_ack=1
            else:self.serial.write(packet)
            self.inner_pending=(self.seq,p,i,d,time.monotonic())
            self.inner_text.set(f'内环 P={p}、I={i/256:g}（板端值 {i}）、D={d} 已发送，等待板端 ACK。')
            self.event('inner_gain_requested',sequence=self.seq,p=p,i_q8=i,d=d)
        except Exception as e:self.inner_text.set('内环 PID 未发送：'+str(e))
        self.buttons()
    def stop(self):
        if not self.connected:return
        self.sine_running=False;self.sine_return_target=None
        try:self.send(3);self.event('stop_requested');self.apply_text.set('已发送停止请求；需观察板上待机/命令0确认，必要时直接按K2/K4。')
        except Exception as e:self.apply_text.set('串口停止未确认，请按板上K2/K4：'+str(e))
    @staticmethod
    def cut_name(sample):
        if sample.config[9]&128 and sample.cut==0:return '起摆能量/速度监测'
        if sample.state==5:
            return {0:'预捕获：角速度阻尼',10:'预捕获：转臂边界制动',6:'预捕获：ADC恢复等待'}.get(sample.cut,CUTS.get(sample.cut,sample.cut))
        return CUTS.get(sample.cut,sample.cut)
    @staticmethod
    def action_name(sample):
        if sample.config[9]&128:
            return {12:'参考能量补摆',13:'能量已足：停止额外补摆',7:'软边界：抑制外冲',14:'臂速限制',9:'等待角度/速度有效'}.get(sample.action,ACTIONS.get(sample.action,sample.action))
        if sample.state==5:
            return '预捕获：臂边界制动' if sample.action==7 else '预捕获：角速度阻尼'
        if sample.config[9]&32:
            return {12:'正向补摆',13:'反向补摆',14:'臂速上限约束',15:'初始激励',5:'摆杆静止/初踢结束'}.get(sample.action,ACTIONS.get(sample.action,sample.action))
        return ACTIONS.get(sample.action,sample.action)
    def receive(self,sample):
        previous_state=self.latest.state if self.latest is not None else None
        self.latest=sample;self.last_rx=time.monotonic();self.count+=1
        if sample.protocol_version!=self.last_protocol_version:
            self.last_protocol_version=sample.protocol_version;self.confirmed_gain=None;self.confirmed_inner=None
            if sample.protocol_version==0x65:
                self.gain_text.set('已识别双环协议 0x65；串口不能区分 R6 至 R14。外环 D=5000 须下载 R13/R14 位流，并等板端 ACK。')
                self.inner_text.set('已识别内环 P/I/D；平衡时同步显示摆杆目标角和实测角。')
            elif sample.protocol_version==0x64:
                self.gain_text.set('已识别 R5 外环 P/I/D 固件；当前增益未回读，点击发送并等 ACK 后才视为生效。')
                self.inner_text.set('已识别 R5 内环 P/I/D 固件；当前增益未回读，点击发送并等 ACK 后才视为生效。')
            elif sample.protocol_version==0x63:
                self.gain_text.set('已识别 R4 外环 P/I/D 固件；当前增益未回读，点击发送并等 ACK 后才视为生效。I 最大累计修正为 ±1 ADC。')
            elif sample.protocol_version==0x62:
                self.gain_text.set('已识别 R3 外环 P/D 固件；I 不受支持且已禁用。输入框是候选值，点击发送并等 ACK 后才视为生效。')
            else:
                self.gain_text.set('当前是旧版位流（协议 0x61），不支持实时外环 PID；本软件不会向它发送调参命令。')
            if sample.protocol_version not in (0x64,0x65):
                self.inner_text.set('当前板端版本不支持实时内环 PID；内环发送按钮已禁用。')
        if previous_state==2 and sample.state!=2:
            self.sine_running=False;self.sine_return_target=None
            self.sine_text.set('已退出直立平衡，正弦位置目标停止发送。')
            self.move_text.set('已退出直立平衡，电机输出已停止；位置目标暂停。再次进入平衡后重新设目标。')
        if sample.state==2 and self.last_balance_trial!=sample.trial_id:
            self.last_balance_trial=sample.trial_id;self.commanded_target=0;self.move_ready_at=0;self.chart_rows.clear();self.chart_marks.clear()
            self.target_var.set('0')
            self.move_text.set('已接管直立；当前位置为本次目标 0 计数。可发送 ±180 计数内的新目标。')
        if not self.initial_read:
            if not self.editor_dirty:self.fill(sample.config)
            self.initial_read=True
        if self.last_active_cfg is not None and self.last_active_cfg!=sample.config and self.pending is None:
            self.apply_text.set('板上实际参数已变化（可能复位），请核对回读；编辑区不会自动下发。')
            self.event('board_config_changed',revision=sample.revision,parameters=dict(zip(NAMES,sample.config)))
        self.last_active_cfg=sample.config
        if self.pending:
            seq,rev,values,sent=self.pending
            if sample.ack_seq==seq:
                if sample.ack==1 and sample.revision==rev and sample.config==values:
                    self.confirmed_applies+=1;self.pending=None
                    try:self.editor_dirty=self.values()!=values
                    except ValueError:self.editor_dirty=True
                    suffix='编辑区另有修改，尚未下发。' if self.editor_dirty else '按板上K1开始；不会自动启动。'
                    self.apply_text.set(f'板上已确认本次提交，参数版本{rev}。'+suffix)
                    self.event('config_confirmed',sequence=seq,revision=rev,parameters=dict(zip(NAMES,values)))
                elif sample.ack in (2,3,4):
                    self.pending=None;self.apply_text.set('板上拒绝：'+ACKS[sample.ack]);self.event('config_rejected',status=sample.ack)
        if self.target_pending:
            seq,target,speed,sent,source=self.target_pending
            if sample.ack_seq==seq:
                if sample.ack==1:
                    step_distance=abs(target-self.commanded_target)
                    self.move_ready_at=(time.monotonic()+step_distance/max(speed,1)+0.5
                                        if source!='sine' else 0)
                    self.commanded_target=target;self.target_pending=None
                    if source=='manual':
                        self.move_text.set(f'板端已接受目标 {target:+d} 计数，本次目标变化 {step_distance} 计数；等待速度曲线预计完成后才能发送下一步。')
                    elif source=='sine_return':
                        self.sine_return_target=None
                        self.sine_text.set(f'正弦已停止，正在返回中心 {target:+d}；到位后可手动移动。')
                    else:self.sine_text.set(f'正弦运行中：板端已确认位置目标 {target:+d}。')
                    self.event('position_confirmed',sequence=seq,target=target,speed=speed,source=source)
                elif sample.ack in (2,3,4):
                    self.target_pending=None
                    reason='需处于直立平衡且摆角不超过 ±16 ADC' if sample.ack==2 else ACKS[sample.ack]
                    if source=='manual':self.move_text.set('板端拒绝位置目标：'+reason)
                    else:
                        self.sine_running=False;self.sine_return_target=None
                        self.sine_text.set('正弦已停止，板端拒绝位置目标：'+reason)
                    self.event('position_rejected',sequence=seq,status=sample.ack,source=source)
        if self.gain_pending:
            seq,p,i,d,version,sent=self.gain_pending
            if sample.ack_seq==seq:
                self.gain_pending=None
                if sample.ack==1:
                    self.confirmed_gain=(p,i,d)
                    detail=f'P={p}、I={i}、D={d}' if i is not None else f'P={p}、D={d}（R3 无 I）'
                    self.gain_text.set(f'板端 ACK 已确认外环 {detail}；运行中生效。输入框后续修改需重新发送。')
                    self.event('outer_gain_confirmed',sequence=seq,p_q8=p,i_q16=i,d_q8=d)
                else:
                    reason=('当前不在允许调参状态：需待机稳定，或直立平衡且摆角接近中心'
                            if sample.ack==2 else ACKS.get(sample.ack,'未知状态'))
                    self.gain_text.set(f'板端拒绝外环调参（ACK {sample.ack}）：'+reason)
                    self.event('outer_gain_rejected',sequence=seq,status=sample.ack)
        if self.inner_pending:
            seq,p,i,d,sent=self.inner_pending
            if sample.ack_seq==seq:
                self.inner_pending=None
                if sample.ack==1:
                    self.confirmed_inner=(p,i,d)
                    self.inner_text.set(f'板端 ACK 已确认内环 P={p}、I={i/256:g}（板端值 {i}）、D={d}；运行中生效。')
                    self.event('inner_gain_confirmed',sequence=seq,p=p,i_q8=i,d=d)
                else:
                    reason=('需待机稳定，或直立平衡且摆角接近中心' if sample.ack==2 else ACKS.get(sample.ack,'未知状态'))
                    self.inner_text.set(f'板端拒绝内环调参（ACK {sample.ack}）：'+reason)
                    self.event('inner_gain_rejected',sequence=seq,status=sample.ack)
        values=sample.config
        if sample.zero_adc is not None:self.last_zero_adc=sample.zero_adc
        self.board_text.set(f'板上回读 v{sample.revision}：时间{values[0]}ms，目标速度{values[6]*100}计数/s，加速上限{values[3]/50:g}%，制动{values[1]}ms/{values[4]/50:g}%，等待{values[2]}ms，次数{values[8]}；模式：'+('速度闭环' if values[9]&8 else '直接PWM'))
        if values[9]&16:
            self.board_text.set(f'板上回读 v{sample.revision}：连续能量补偿；初踢{min(values[0],80)}ms，臂速上限{values[6]*100}计数/s，力度上限{values[3]/50:g}%，段数上限{values[8]}；没有固定200ms等待；自动平衡'+('开启' if values[9]&1 else '关闭'))
        self.live_text.set(('演示数据，未打开串口！  ' if self.demo else '')+f'试验{sample.trial_id} / {STATES[sample.state]} / {self.action_name(sample)}；当前步{sample.step_ms}ms，最近实际驱动{sample.actual_accel_ms}ms；原因：{self.cut_name(sample)}\n'+f'摆杆{sample.angle_x2/2:g}ADC，臂位置{sample.encoder}，速度{sample.arm_speed*100}计数/s，PWM命令{sample.motor}；停机：{INHIBITS.get(sample.inhibit,sample.inhibit)}')
        if values[9]&16 or sample.state==2:
            stage='原直立角度/位置双环闭环运行' if sample.state==2 else self.action_name(sample)
            self.live_text.set(('演示，非实板！ ' if self.demo else '')+f'试验{sample.trial_id} / {STATES[sample.state]} / {stage}；补能方向段{sample.pulse_index}；能量差估计Q8={sample.energy_error_q8}（不是焦耳）\n'+f'摆杆{sample.angle_x2/2:g}ADC，臂位置{sample.encoder}，速度{sample.arm_speed*100}计数/s，命令{sample.motor}；停机：{INHIBITS.get(sample.inhibit,sample.inhibit)}')
        if values[9]&32:
            self.board_text.set(f'板上回读 v{sample.revision}：LAB3 速度方向起摆；初踢{min(values[0],80)}ms，力度{values[3]/50:g}%，臂速上限{values[6]*100}计数/s；每5ms判断摆杆方向，无固定等待；自动平衡'+('开启' if values[9]&1 else '关闭'))
            stage='原直立角度/位置双环闭环运行' if sample.state==2 else self.action_name(sample)
            self.live_text.set(('演示，非实板！ ' if self.demo else '')+f'试验{sample.trial_id} / {STATES[sample.state]} / {stage}；方向段{sample.pulse_index}；5ms摆杆差分{sample.relay_delta_x2}半ADC；诊断：{self.cut_name(sample)}\n'+f'摆杆{sample.angle_x2/2:g}ADC，臂位置{sample.encoder}，臂速{sample.arm_speed*100}计数/s，命令{sample.motor}；停机：{INHIBITS.get(sample.inhibit,sample.inhibit)}')
        if values[9]&32:
            self.board_text.set(self.board_text.get()+'；顶部预捕获'+('开启' if values[9]&64 else '关闭'))
        if sample.state==5 and sample.cut==10:
            self.live_text.set(self.live_text.get().replace('初始激励结束','预捕获：转臂接近硬边界，限制外冲'))
        if values[9]&128:
            cal='标定中：继续保持直立静止' if sample.zero_busy else '已标定' if sample.zero_valid else '标定失败：保持静止重新按K3' if sample.zero_failed else '未标定：扶直静止按K3'
            self.board_text.set(f'LAB4 板上回读 v{sample.revision}：力度{values[3]/50:g}%，臂速上限{values[6]*100}计数/s，软行程{values[7]}；零位{self.last_zero_adc if self.last_zero_adc is not None else "--"} ADC / {cal}')
            speed=f'{sample.reference_speed_rad_s:g} rad/s' if sample.state==1 else '--（只在起摆估计）'
            stage='原直立双环' if sample.state==2 else self.action_name(sample)
            self.live_text.set(('演示，非实板！ ' if self.demo else '')+f'{STATES[sample.state]} / {stage}；摆杆{sample.angle_x2/2:g} ADC，起摆角速度{speed}；诊断{self.cut_name(sample)}\n'+f'转臂{sample.encoder}计数 / {sample.arm_speed*100}计数/s；PWM命令{sample.motor}；停机{INHIBITS.get(sample.inhibit,sample.inhibit)}')
            if sample.position_target_counts is not None:
                self.live_text.set(self.live_text.get()+f'\n横杆位置：目标 {sample.position_target_counts}、实际 {sample.encoder}、误差 {sample.position_error_counts:+d} 计数；已确认设定 {self.commanded_target:+d} 计数')
            if sample.inner_target_adc is not None:
                self.live_text.set(self.live_text.get()+f'；摆杆角度：目标 {sample.inner_target_adc:g}、实际 {sample.angle_x2/2:g} ADC')
        t=self.last_rx-self.started;self.rows.append((t,sample))
        if sample.state==2:self.chart_rows.append((t,sample))
        self.writer.writerow((f'{t:.6f}',sample.seq,STATES[sample.state],self.action_name(sample),self.cut_name(sample),INHIBITS.get(sample.inhibit,sample.inhibit),sample.angle_x2/2,sample.encoder,sample.motor,sample.arm_speed*100,None if values[9]&128 else sample.step_ms,None if values[9]&176 else sample.actual_accel_ms,None if values[9]&128 else sample.pulse_index,sample.trial_id,sample.revision,sample.ack_seq,sample.ack,int(sample.config_safe),self.decoder.crc_errors,self.decoder.gaps,sample.rx_crc_errors,sample.energy_error_q8,sample.relay_delta_x2,sample.cut if values[9]&160 else None,self.last_zero_adc,int(sample.zero_valid),int(sample.zero_busy),int(sample.zero_failed),sample.reference_speed_rad_s,sample.position_error_counts,sample.position_target_counts,self.commanded_target if sample.state==2 else None,sample.protocol_version,self.confirmed_gain[0] if self.confirmed_gain else None,self.confirmed_gain[1] if self.confirmed_gain else None,self.confirmed_gain[2] if self.confirmed_gain else None,self.confirmed_inner[0] if self.confirmed_inner else None,self.confirmed_inner[1] if self.confirmed_inner else None,self.confirmed_inner[2] if self.confirmed_inner else None,sample.inner_target_adc,sample.position_target_counts,*sample.config))
    def poll(self):
        now=time.monotonic()
        if self.connected:
            try:
                if self.pending and now-self.pending[3]>1.0:
                    self.event('config_confirmation_timeout');self.pending=None
                    self.apply_text.set('下发未确认：不要开始试验，先核对板上回读；不会自动重发。')
                if self.target_pending and now-self.target_pending[3]>0.5:
                    self.event('position_confirmation_timeout',sequence=self.target_pending[0])
                    source=self.target_pending[4]
                    if source!='manual':
                        self.sine_running=False;self.sine_return_target=None
                        self.sine_text.set('正弦已停止：位置目标 ACK 超时，生效状态未知。')
                    self.target_pending=None
                    if source=='manual':
                        self.move_text.set('位置目标未确认。请查看板端状态后重新发送；不会自动重发。')
                if self.gain_pending and now-self.gain_pending[5]>0.5:
                    self.event('outer_gain_confirmation_timeout',sequence=self.gain_pending[0])
                    self.gain_pending=None
                    self.gain_text.set('外环 PID 未获板端确认，生效状态未知；请核对连接后重新发送。')
                if self.inner_pending and now-self.inner_pending[4]>0.5:
                    self.event('inner_gain_confirmation_timeout',sequence=self.inner_pending[0])
                    self.inner_pending=None
                    self.inner_text.set('内环 PID 未获板端确认，生效状态未知；请核对连接后重新发送。')
                if self.pending is None and self.target_pending is None and self.gain_pending is None and self.inner_pending is None and now-self.last_query>=0.2:
                    self.send(2);self.last_query=now
                if self.demo:
                    self.demo_seq=(self.demo_seq+1)&255
                    p=Sample(self.demo_seq,16,0,0,0,0,self.demo_ack_seq,self.demo_ack,self.demo_safe,
                             round(450+20*math.sin(now*3)),0,0,0,0,self.demo_revision,self.demo_cfg,0,0,0,0,0)
                    incoming=pack(replace(p,protocol_version=self.demo_version))
                else:incoming=self.serial.read()
                for sample in self.decoder.feed(incoming):self.receive(sample)
                self.auto_tune(now)
                self.update_sine(now)
                if now-self.last_flush>=0.5:
                    self.log.flush();self.last_flush=now
                if not self.fresh():self.note.set('遥测已过期/未收到试验台固件。禁止下发；通信中断1秒固件停止。检查新位流、串口与L20接收线。')
                else:self.note.set(f'CRC错误{self.decoder.crc_errors}，序号缺口{self.decoder.gaps}。日志：{self.base}；若已下载R14，横杆硬行程停机关闭；K2停止、K4急停。')
            except Exception as e:
                self.apply_text.set('通信异常，请确认机构停止：'+str(e));self.disconnect()
        self.buttons()
        if now-self.last_draw>=0.1:self.draw();self.last_draw=now
        self.root.after(20,self.poll)
    def open_large_chart(self):
        if self.large_chart_window is not None and self.large_chart_window.winfo_exists():
            self.large_chart_window.lift();self.large_chart_window.focus_set();return
        window=tk.Toplevel(self.root)
        self.large_chart_window=window
        window.title('J280 实时大曲线 · 目标与实际')
        width=min(1600,max(1050,int(self.root.winfo_screenwidth()*0.82)))
        height=min(1000,max(650,int(self.root.winfo_screenheight()*0.75)))
        window.geometry(f'{width}x{height}')
        window.minsize(900,550)
        controls=ttk.Frame(window,padding=8);controls.pack(fill='x')
        ttk.Label(controls,text='青色：实际　黄色：目标　紫色竖线：内环 PID 已确认　绿色竖线：外环 PID 已确认').pack(side='left')
        self.large_mode=tk.StringVar(value='both')
        for label,value in (('两条曲线','both'),('只看摆杆角','angle'),('只看横杆位置','position')):
            ttk.Radiobutton(controls,text=label,variable=self.large_mode,value=value,command=self.draw).pack(side='left',padx=8)
        self.large_window_var=tk.StringVar(value='20')
        ttk.Label(controls,text='时间窗').pack(side='left',padx=(16,4))
        ttk.Combobox(controls,textvariable=self.large_window_var,values=('5','10','20','30','60'),
                     state='readonly',width=5).pack(side='left')
        self.large_chart_canvas=tk.Canvas(window,bg='#111827',highlightthickness=0)
        self.large_chart_canvas.pack(fill='both',expand=True,padx=8,pady=(0,8))
        window.protocol('WM_DELETE_WINDOW',self.close_large_chart)
        self.draw()
    def close_large_chart(self):
        if self.large_chart_window is not None:self.large_chart_window.destroy()
        self.large_chart_window=None;self.large_chart_canvas=None
    def draw(self):
        self.draw_chart(self.canvas,self.window_var,'both')
        if self.large_chart_canvas is not None and self.large_chart_canvas.winfo_exists():
            self.draw_chart(self.large_chart_canvas,self.large_window_var,self.large_mode.get())
    def draw_chart(self,canvas,window_var,mode):
        canvas.delete('all');w=max(canvas.winfo_width(),200);h=max(canvas.winfo_height(),180)
        if not self.chart_rows:
            canvas.create_text(w/2,h/2,text='等待进入直立平衡；曲线会从本次接管时开始记录',fill='#cbd5e1')
            return
        window=int(window_var.get())
        last=self.chart_rows[-1][0];left=max(self.chart_rows[0][0],last-window);span=max(1,last-left)
        data=[row for row in self.chart_rows if row[0]>=left]
        stride=max(1,len(data)//max(1,int(w//2)))
        plotted=data[::stride]
        if plotted[-1]!=data[-1]:plotted.append(data[-1])
        panels=[]
        if mode in ('both','angle'):
            panels.append(('摆杆角度 · ADC',lambda s:s.angle_x2/2,lambda s:s.inner_target_adc,12))
        if mode in ('both','position'):
            panels.append(('横杆位置 · 编码器计数',lambda s:s.encoder,lambda s:s.position_target_counts,40))
        for n,(label,actual,target,min_span) in enumerate(panels):
            top=n*h/len(panels);bottom=(n+1)*h/len(panels)
            y0=top+42;y1=bottom-28
            measurements=[v for _,s in data for v in (actual(s),target(s)) if v is not None]
            if not measurements:continue
            lo=min(measurements);hi=max(measurements);mid=(lo+hi)/2
            half=max(min_span/2,(hi-lo)*0.65);lo=mid-half;hi=mid+half
            current=data[-1][1];av=actual(current);tv=target(current)
            legend=f'实际 {av:g}    目标 {tv:g}    偏差 {av-tv:+g}' if tv is not None else f'实际 {av:g}    目标需 0x65 位流'
            canvas.create_text(12,top+15,text=label,fill='#e5e7eb',anchor='w',font=('Microsoft YaHei UI',10,'bold'))
            canvas.create_text(w-12,top+15,text=legend,fill='#e5e7eb',anchor='e')
            for fraction in (0,0.5,1):
                value=lo+(hi-lo)*fraction;y=y1-fraction*(y1-y0)
                canvas.create_line(76,y,w-12,y,fill='#334155')
                canvas.create_text(70,y,text=f'{value:.1f}' if label.startswith('摆杆') else f'{value:.0f}',fill='#94a3b8',anchor='e')
            for fn,color,width in ((target,'#facc15',2),(actual,'#38bdf8',2)):
                coords=[]
                for t,s in plotted:
                    v=fn(s)
                    if v is None:continue
                    x=76+(t-left)/span*(w-88)
                    y=y1-(v-lo)/(hi-lo)*(y1-y0)
                    coords.extend((x,y))
                if len(coords)>=4:canvas.create_line(*coords,fill=color,width=width)
            canvas.create_text(78,bottom-8,text=f'{max(0,last-left):.1f} s / 最近 {window} s',fill='#94a3b8',anchor='w')
        if canvas is self.large_chart_canvas:
            visible=[mark for mark in self.chart_marks if left<=mark[0]<=last]
            for index,(t,title,p,i,d) in enumerate(visible):
                x=76+(t-left)/span*(w-88)
                color='#a78bfa' if title=='内环' else '#4ade80'
                canvas.create_line(x,33,x,h-28,fill=color,dash=(4,5),width=2)
                canvas.create_text(x+4,34+(index%3)*17,text=f'{title} P{p} I{i} D{d}',
                                   fill=color,anchor='nw',font=('Consolas',9))
    def disconnect(self):
        self.sine_running=False;self.sine_return_target=None
        if self.serial:
            try:self.send(3)
            except Exception:pass
            self.serial.close();self.serial=None
        self.event('disconnected',frames=self.count)
        if self.log:self.log.flush();self.log.close();self.log=None
        if self.event_stream:self.event_stream.close();self.event_stream=None
        self.connected=False;self.pending=None;self.target_pending=None;self.gain_pending=None;self.inner_pending=None
        self.connect_button.configure(text='连接并记录');self.buttons()
    def close(self):self.disconnect();self.root.destroy()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true')
    parser.add_argument('--self-test',action='store_true');parser.add_argument('--verify',type=Path)
    parser.add_argument('--verify-r3',type=Path);parser.add_argument('--verify-r4',type=Path);parser.add_argument('--verify-r5',type=Path)
    args=parser.parse_args()
    if args.self_test:self_test();return
    root=tk.Tk()
    verify_path=args.verify or args.verify_r3 or args.verify_r4 or args.verify_r5
    if verify_path:root.withdraw()
    app=Lab(root,args.demo or bool(verify_path),verify_path.parent if verify_path else None,
            0x62 if args.verify_r3 else 0x63 if args.verify_r4 else 0x64 if args.verify_r5 else 0x65)
    if args.demo or verify_path:app.connect_toggle()
    if verify_path:
        def apply_test():app.fields[0].set('160');app.apply()
        def tune_test():
            app.outer_p_var.set('16');app.outer_i_var.set('4');app.outer_d_var.set('512');app.apply_outer_gains()
        def inner_test():
            app.inner_p_var.set('240');app.inner_i_var.set('0.0625');app.inner_d_var.set('450');app.apply_inner_gains()
        def chart_test():
            s=replace(app.latest,state=2,trial_id=1,angle_x2=1510,step_ms=1512,
                      encoder=100,actual_accel_ms=5,pulse_index=32768)
            app.receive(s);app.receive(replace(s,angle_x2=1514,encoder=101))
            app.draw()
        def finish():
            result=dict(status='FAIL',frames=app.count,confirmed_applies=app.confirmed_applies)
            try:
                assert app.count>10 and app.confirmed_applies==1 and app.latest.config[0]==160
                expected=(16,None,512) if args.verify_r3 else (16,4,512)
                assert app.confirmed_gain==expected and app.latest.protocol_version==app.demo_version
                if args.verify:
                    assert app.confirmed_inner==(240,16,450)
                    assert len(app.chart_rows)==2 and app.chart_rows[-1][1].inner_target_adc==756
                    assert app.chart_rows[-1][1].position_target_counts==106
                    assert len(app.canvas.find_all())>5
                assert app.latest.motor==0 and app.latest.state==0
                assert app.base.with_suffix('.csv').is_file()
                result.update(status='PASS',csv=str(app.base.with_suffix('.csv')),config=list(app.latest.config),outer_gains=app.confirmed_gain,inner_gains=app.confirmed_inner)
            except Exception as e:result['error']=repr(e)
            finally:app.disconnect();verify_path.parent.mkdir(parents=True,exist_ok=True);verify_path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');root.destroy()
        root.after(300,apply_test);root.after(650,tune_test)
        if args.verify:
            root.after(1050,inner_test);root.after(1400,chart_test)
        root.after(1800,finish)
    root.mainloop()

if __name__=='__main__':main()
