"""Own GUI only, simulated board only, no real COM/motor writes."""
from dataclasses import replace
import ctypes
from pathlib import Path
import sys
import time
import tkinter as tk
from swing_lab import Lab
from lab_serial import DCB,Timeouts
from lab_protocol import DEFAULTS

def main(folder):
    root=tk.Tk();root.withdraw();app=Lab(root,demo=True,log_root=folder)
    def pump(seconds=0.15):
        end=time.monotonic()+seconds
        while time.monotonic()<end:root.update();time.sleep(0.005)
    try:
        assert (app.outer_p_var.get(),app.outer_i_var.get(),app.outer_d_var.get())==('15','19','5000')
        assert (app.inner_p_var.get(),app.inner_i_var.get(),app.inner_d_var.get())==('115','0.25','520')
        assert app.move_limit()==60
        app.move_step_var.set('180');assert app.move_limit()==180
        app.move_step_var.set('181')
        try:app.move_limit();assert False,'oversized step accepted'
        except ValueError:pass
        app.move_step_var.set('60')
        sent=[];original_move_to=app.move_to;app.move_to=sent.append
        app.commanded_target=150;app.move_by(1);assert sent==[180]
        app.commanded_target=180;app.move_by(1);assert sent==[180]
        app.commanded_target=0;app.move_to=original_move_to
        assert ctypes.sizeof(DCB)==28 and ctypes.sizeof(Timeouts)==20
        assert app.values()[9]==129 and app.strategybox.current()==0
        app.strategybox.current(1);app.choose_strategy();assert app.values()[9]&48==32
        app.strategybox.current(2);app.choose_strategy();assert app.values()[9]&48==16
        app.strategybox.current(3);app.choose_strategy();assert app.values()[9]&56==8
        app.strategybox.current(4);app.choose_strategy();assert app.values()[9]&56==0
        app.relay_preset();assert app.values()==DEFAULTS
        app.energy_preset();assert app.values()[9]==17
        app.preset(False);assert app.values()[9]&48==0
        app.relay_preset()
        app.connect_toggle();pump()
        app.fields[0].set('160');app.fields[6].set('1000')
        assert app.editor_dirty and app.values()[6]==10
        app.apply();assert app.pending is not None;pump()
        assert app.confirmed_applies==1 and app.latest.config[0]==160 and app.latest.config[6]==10
        app.fields[0].set('180');assert app.editor_dirty and '尚未下发' in app.apply_text.get()
        # Race: state was safe at host send, but board becomes busy before commit.
        app.demo_safe=False;app.latest=replace(app.latest,config_safe=True);app.apply();pump()
        assert app.pending is None and app.confirmed_applies==1 and app.latest.config[0]==160
        assert '拒绝' in app.apply_text.get(),app.apply_text.get()
        assert str(app.apply_button['state'])=='disabled',str(app.apply_button['state'])
        # Board reset/default change must not replay the edited settings.
        app.demo_safe=True;app.demo_cfg=DEFAULTS;app.demo_revision=0;app.demo_ack=0;pump()
        assert app.latest.config==DEFAULTS and app.fields[0].get()=='180'
        assert '变化' in app.apply_text.get()
        app.stop();pump();assert app.latest.state==0 and app.latest.motor==0
        assert app.base.with_suffix('.csv').is_file() and app.base.with_suffix('.jsonl').is_file()
        legacy=list(app.latest.config);legacy[6]=16;legacy[7]=180;legacy[9]=97
        pre=replace(app.latest,config=tuple(legacy),state=5,config_safe=False,action=7,cut=10,actual_accel_ms=8)
        app.receive(pre);app.buttons()
        assert '顶部预捕获' in app.live_text.get() and '边界' in app.live_text.get()
        assert str(app.apply_button['state'])=='disabled'
        import csv
        app.log.flush()
        with app.base.with_suffix('.csv').open(encoding='utf-8-sig',newline='') as f:
            rows=list(csv.DictReader(f))
        assert rows and all(None not in row for row in rows)
        assert all(row['energy_error_q8']=='' and row['actual_accel_ms']=='' for row in rows)
        assert rows[-1]['state']=='顶部预捕获' and rows[-1]['relay_delta_x2']=='8' and '边界' in rows[-1]['cut_reason']
        ref=replace(app.latest,config=DEFAULTS,state=1,step_ms=775,pulse_index=32769,actual_accel_ms=512,cut=12)
        app.receive(ref);assert '775 ADC' in app.board_text.get() and '已标定' in app.board_text.get()
        assert '2 rad/s' in app.live_text.get()
        app.log.flush()
        with app.base.with_suffix('.csv').open(encoding='utf-8-sig',newline='') as f:row=list(csv.DictReader(f))[-1]
        assert None not in row and row['zero_adc']=='775' and row['zero_valid']=='1' and row['pole_speed_rad_s']=='2.0'
        assert row['step_ms']=='' and row['pulse_index']=='' and row['relay_delta_x2']==''
        assert str(app.spins[0]['state'])=='disabled' and str(app.spins[3]['state'])=='normal'
        # The enlarged chart is an independent, resizable view of the same
        # telemetry and shows only confirmed PID changes.
        chart_sample=replace(app.latest,state=2,protocol_version=0x65,trial_id=1,
                             angle_x2=1482,step_ms=1480,encoder=98,actual_accel_ms=2)
        app.chart_rows.append((0.0,chart_sample))
        app.chart_rows.append((0.1,replace(chart_sample,angle_x2=1484,encoder=99,actual_accel_ms=1)))
        app.chart_marks.append((0.05,'外环',17,0,400))
        app.open_large_chart();root.update();app.draw()
        assert app.large_chart_window is not None and app.large_chart_window.winfo_exists()
        assert len(app.large_chart_canvas.find_all())>8
        app.large_mode.set('position');app.draw()
        assert len(app.large_chart_canvas.find_all())>5
        app.close_large_chart();assert app.large_chart_canvas is None
        print('SWING_LAB4_UI_PASS zero_readback physical_speed legacy_CSV disabled_apply exclusive_strategies presets readback busy_race reset_no_replay native_DCB')
    finally:app.close()

if __name__=='__main__':main(Path(sys.argv[1]))
