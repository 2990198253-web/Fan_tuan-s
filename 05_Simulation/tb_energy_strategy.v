`timescale 1ns/1ps
module tb_energy_strategy;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,en=1,up=0,phase=1,pv=1,av=1,top=0;
    reg [223:0] cfg;reg signed [32:0] pos=0,vel=0;
    reg signed [31:0] energy=512;reg signed [15:0] cosine=-256;
    wire signed [15:0] cmd;wire [3:0] action,cut;wire [15:0] step,last,segments;wire done;
    swing_energy_controller dut(.clk(clk),.rst_n(rst),.enable(en),.update(up),.config_words(cfg),
        .displacement(pos),.arm_velocity(vel),.energy_error_q8(energy),.cosine(cosine),
        .phase_positive(phase),.phase_valid(pv),.angle_valid(av),.capture_zone(top),
        .command(cmd),.action(action),.cut_reason(cut),.step_ms(step),.last_accel_ms(last),.pulse_index(segments),.done(done));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    task ticks;input integer n;integer i;begin for(i=0;i<n;i=i+1)tick();end endtask
    task reset_case;begin
        rst=0;en=1;up=0;phase=1;pv=1;av=1;top=0;pos=0;vel=0;energy=512;cosine=-256;
        cfg={16'd20,16'd1000,16'd15000,16'd2,16'd17,16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
        repeat(3)@(negedge clk);rst=1;
    end endtask
    initial begin
        $dumpfile("smooth_results/lab2_energy.vcd");$dumpvars(0,dut);
        reset_case();tick();if(cmd!=2500||action!=15)$fatal(1,"Initial seed missing");
        ticks(4);if(cmd!=0||last!=40)$fatal(1,"Bounded seed timing");
        ticks(50);if(cmd!=2500||action!=12||done)$fatal(1,"Fixed pulse/brake/wait still interrupts continuous pump");
        phase=0;ticks(3);if(cmd!=-2500||segments!=2)$fatal(1,"Measured phase did not reverse pump");
        energy=-512;ticks(2);if(cmd!=2500||action!=13)$fatal(1,"Excess energy did not reverse extraction");
        energy=0;tick();if(cmd!=0||action!=13)$fatal(1,"Energy hold still injecting");
        energy=512;phase=1;ticks(3);vel=16;tick();
        if(cmd>0||cmd<=-2500||action!=14)$fatal(1,"Soft speed envelope became full reverse relay");
        phase=0;ticks(3);if(cmd>=0)$fatal(1,"Envelope blocked opposing energy torque");
        vel=24;tick();if(action!=6||cmd!=-2500)$fatal(1,"Hard overspeed lost");
        reset_case();tick();pos=160;vel=10;tick();if(action!=7||cmd!=-2500)$fatal(1,"Travel prediction lost");
        ticks(21);if(!done||cmd!=0||cut!=7)$fatal(1,"Failed braking not terminal");
        reset_case();av=0;tick();if(cmd!=0||action!=9)$fatal(1,"ADC rail allowed initial seed");
        reset_case();tick();av=0;vel=2;tick();if(cmd!=-2500||action!=9)$fatal(1,"Invalid ADC did not damp arm");
        reset_case();tick();top=1;vel=2;tick();if(cmd!=-2500||action!=8)$fatal(1,"Top capture not supported");
        reset_case();tick();ticks(4);cfg[10*16+:16]=1;phase=0;ticks(2);if(cmd>=0)$fatal(1,"One-sample confirmation");
        en=0;@(negedge clk);if(cmd!=0)$fatal(1,"Disable");
        reset_case();cfg[9*16+:16]=21;tick();if(cmd!=-2500)$fatal(1,"Reverse seed");
        ticks(12);if(cmd!=-2500)$fatal(1,"Energy polarity flag only reversed seed");
        $display("SWING_LAB2_ENERGY_PASS continuous500ms phase energy_add_extract speed_envelope hard_guard rails top stop");$finish;
    end
    initial begin #1000000;$fatal(1,"Energy test timeout");end
endmodule
