`timescale 1ns/1ps
module tb_lab_controller;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,en=1,up=0,phase=1,pv=1,av=1,top=0;
    reg [223:0] cfg;reg signed [32:0] pos=0,vel=0;
    wire signed [15:0] cmd;wire [3:0] action,cut;
    wire [15:0] step,last,pulses;wire done;
    swing_lab_controller dut(.clk(clk),.rst_n(rst),.enable(en),.update(up),.config_words(cfg),
        .displacement(pos),.arm_velocity(vel),.phase_positive(phase),.phase_valid(pv),
        .angle_valid(av),.capture_zone(top),.command(cmd),.action(action),.cut_reason(cut),
        .step_ms(step),.last_accel_ms(last),.pulse_index(pulses),.done(done));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    task ticks;input integer n;integer i;begin for(i=0;i<n;i=i+1)tick();end endtask
    task reset_case;begin rst=0;en=1;up=0;pos=0;vel=0;pv=1;phase=1;av=1;top=0;
        cfg={16'd20,16'd1000,16'd15000,16'd2,16'd0,16'd1,16'd180,16'd8,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd120};
        repeat(3)@(negedge clk);rst=1;end endtask
    initial begin
        $dumpfile("smooth_results/lab_controller.vcd");$dumpvars(0,dut);
        reset_case();tick();if(cmd!=2500)$fatal(1,"Direct PWM start");
        ticks(11);if(action!=2||cmd!=2500)$fatal(1,"Unexpected early deceleration before120ms");
        vel=2;tick();if(action!=4||cmd!=-2500||cut!=1||last!=120)$fatal(1,"Timed acceleration duration");
        vel=0;tick();ticks(2);if(!done||cmd!=0||pulses!=1)$fatal(1,"Single pulse completion");
        en=0;@(negedge clk);if(cmd!=0||last!=120||pulses!=1)$fatal(1,"Completed measurements not retained");
        reset_case();cfg[0+:16]=200;cfg[9*16+:16]=8;tick();
        if(cmd!=2160)$fatal(1,"Speed target used as maximum PWM instead of velocity control");
        vel=8;pos=20;ticks(8);
        if(action!=11||cmd!=560||done||cut!=0)$fatal(1,"Reaching target speed prematurely braked");
        vel=10;tick();if(cmd>=560||action==4)$fatal(1,"Speed regulation missing");
        vel=8;ticks(11);if(cut!=1||last!=200)$fatal(1,"Speed hold did not last configured duration");
        reset_case();tick();pos=141;vel=1;tick();if(cut!=2||last!=10)$fatal(1,"Distance cause not reported");
        reset_case();tick();vel=23;tick();if(action==4||cut!=0)$fatal(1,"Old speed10 cutoff still active");
        vel=24;tick();if(cut!=9||action!=6||cmd!=-2500)$fatal(1,"Fixed overspeed safety");
        reset_case();tick();pos=160;vel=10;tick();if(cut!=4||action!=7)$fatal(1,"20ms predictive travel guard");
        reset_case();tick();av=0;tick();if(cut!=6||action!=9)$fatal(1,"Invalid ADC safety");
        reset_case();cfg[9*16+:16]=1;tick();top=1;vel=2;tick();if(cut!=5||action!=8)$fatal(1,"Capture approach safety");
        reset_case();cfg[0+:16]=10;cfg[1*16+:16]=20;tick();vel=3;tick();ticks(2);
        if(!done||cmd!=0||cut!=7)$fatal(1,"Brake timeout not terminal");
        reset_case();cfg[8*16+:16]=2;cfg[2*16+:16]=20;cfg[0+:16]=10;tick();tick();tick();ticks(2);
        if(pulses!=2||cmd!=-2500)$fatal(1,"Timed alternating second pulse");
        reset_case();cfg[8*16+:16]=2;cfg[9*16+:16]=2;cfg[2*16+:16]=20;cfg[0+:16]=10;
        tick();tick();tick();ticks(10);if(pulses!=1)$fatal(1,"Unconfirmed phase repeated kick");
        phase=0;ticks(2);if(pulses!=2||cmd!=-2500)$fatal(1,"Confirmed opposite phase did not pump");
        en=0;@(negedge clk);if(cmd!=0)$fatal(1,"Disable gate");
        reset_case();tick();ticks(5);en=0;@(negedge clk);
        if(cmd!=0||last!=50||cut!=8)$fatal(1,"External stop lost actual drive duration");
        $display("SWING_LAB_CONTROLLER_PASS adjustable_time speed_hold PWM distance speed travel phase single repeat brake_timeout stop");$finish;
    end
    initial begin #1000000;$fatal(1,"Lab controller timeout");end
endmodule
