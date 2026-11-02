`timescale 1ns/1ps
module tb_impulse_swing;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,en=1,up=0,phase=1,valid=0,av=1,top=0;
    reg signed [32:0] pos=0,vel=0;
    wire signed [15:0] cmd,reverse_cmd;
    wire [3:0] action,reverse_action;
    impulse_swing_controller dut(.clk(clk),.rst_n(rst),.enable(en),.update(up),
        .displacement(pos),.arm_velocity(vel),.phase_positive(phase),.phase_valid(valid),
        .angle_valid(av),.capture_zone(top),.command(cmd),.action(action));
    impulse_swing_controller #(.RETURN_POLARITY(1),.DRIVE_POLARITY(1)) reversed(
        .clk(clk),.rst_n(rst),.enable(en),.update(up),.displacement(pos),.arm_velocity(vel),
        .phase_positive(phase),.phase_valid(valid),.angle_valid(av),.capture_zone(top),
        .command(reverse_cmd),.action(reverse_action));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    task ticks;input integer n;integer i;begin for(i=0;i<n;i=i+1)tick();end endtask
    task reset_case;begin rst=0;en=1;up=0;pos=0;vel=0;phase=1;valid=0;av=1;top=0;
        repeat(3)@(negedge clk);rst=1;end endtask
    initial begin
        $dumpfile("smooth_results/r5_impulse.vcd");$dumpvars(0,dut);
        reset_case();tick();
        if(cmd!=4000||reverse_cmd!=-4000)$fatal(1,"Full bounded kick or startup polarity");
        vel=4;pos=20;ticks(3);if(cmd!=4000)$fatal(1,"Position servo still limiting kick");
        vel=10;pos=50;tick();
        if(cmd!=-4000||action!=4||reverse_cmd!=4000)$fatal(1,"Speed threshold active brake/polarity");
        vel=3;tick();if(cmd!=-4000)$fatal(1,"Brake did not persist while moving");
        vel=0;tick();if(cmd!=0||action!=5)$fatal(1,"Brake did not release at low speed");
        ticks(8);phase=0;valid=1;tick();if(cmd!=0)$fatal(1,"Unconfirmed phase kicked");
        tick();if(cmd!=-4000||action!=3)$fatal(1,"Opposite phase did not kick");
        vel=-5;pos=-35;tick(); // 85 counts from this kick's origin50.
        if(cmd!=4000||action!=4)$fatal(1,"Distance cap failed");
        en=0;@(negedge clk);if(cmd!=0)$fatal(1,"Disable output not zero");
        reset_case();tick();vel=2;ticks(8);
        if(cmd!=-4000||action!=4)$fatal(1,"80ms kick duration cap");
        vel=0;tick();ticks(51);if(cmd!=-4000)$fatal(1,"Motionless reseed");
        reset_case();tick();pos=100;vel=8;tick();
        if(cmd!=-4000||action!=7)$fatal(1,"Predictive outward guard braking");
        vel=0;tick();if(cmd>=0||cmd < -1400)$fatal(1,"Bounded guard return");
        pos=20;tick();if(cmd!=0||action!=5)$fatal(1,"Return completion");
        reset_case();tick();vel=17;tick();if(cmd!=-4000||action!=6)$fatal(1,"Overspeed brake");
        reset_case();tick();vel=3;top=1;tick();if(cmd!=-4000||action!=8)$fatal(1,"Top brake");
        top=0;av=0;tick();if(cmd!=-4000||action!=9)$fatal(1,"Invalid angle brake");
        vel=0;ticks(80);if(cmd!=0)$fatal(1,"Invalid angle allowed reseed");
        $display("V6R5_IMPULSE_PASS kick4000 brake distance speed duration phase reseed guard polarity disable rail top");
        $finish;
    end
    initial begin #1000000;$fatal(1,"Impulse test timeout");end
endmodule
