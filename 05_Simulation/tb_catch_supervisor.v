`timescale 1ns/1ps
module tb_catch_supervisor;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,up=0,start=0,stop=0,estop=0,fresh=1,illegal=0;
    reg [10:0] angle=1250;reg signed [31:0] pos=0;reg [223:0] cfg;
    wire signed [15:0] cmd;wire [2:0] state;wire capture;wire [3:0] reason;
    swing_up_supervisor #(.LAB_ENABLE(1),.VELOCITY_TICKS(20),.PRECAPTURE_TIMEOUT_TICKS(120)) dut(
        .clk(clk),.rst_n(rst),.update(up),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(fresh),.encoder_illegal(illegal),.angle_x2(angle),.position(pos),.lab_config_words(cfg),
        .swing_command(cmd),.state(state),.capture_balance(capture),.reason(reason));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    task ticks;input integer n;integer i;begin for(i=0;i<n;i=i+1)tick();end endtask
    task restart;begin rst=0;up=0;stop=0;estop=0;fresh=1;illegal=0;angle=1250;pos=0;
        cfg={16'd20,16'd1000,16'd15000,16'd2,16'd97,16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
        repeat(3)@(negedge clk);rst=1;start=1;@(negedge clk);start=0;ticks(40);end endtask
    task enter;begin angle=1290;ticks(20);angle=1330;ticks(20);
        if(state!=5||!dut.running||!dut.swinging||dut.balancing)$fatal(1,"Precapture does not enter while still running");
    end endtask
    initial begin
        restart();enter();angle=1390;ticks(10);angle=1450;ticks(10);angle=1470;ticks(10);angle=1510;ticks(10);
        if(state!=5)$fatal(1,"High-speed passage was falsely balanced");
        ticks(20);if(state!=5)$fatal(1,"Strict two-sample confirmation relaxed");
        ticks(20);if(state!=2)$fatal(1,"Slow pole not handed off");
        restart();enter();stop=1;#1;if(cmd!=0)$fatal(1,"Precapture raw stop");
        restart();enter();estop=1;#1;if(cmd!=0)$fatal(1,"Precapture raw estop");
        restart();enter();illegal=1;#1;if(cmd!=0)$fatal(1,"Precapture encoder gate");
        restart();enter();fresh=0;#1;if(cmd!=0)$fatal(1,"Precapture stale ADC gate");
        restart();enter();pos=240;#1;if(cmd!=0)$fatal(1,"Precapture positive hard limit");
        restart();enter();pos=-240;#1;if(cmd!=0)$fatal(1,"Precapture negative hard limit");
        restart();enter();angle=0;#1;if(cmd!=0)$fatal(1,"Precapture rail drive");
        ticks(240);if(state!=3||reason!=11)$fatal(1,"ADC fault lost through catch-to-swing");
        restart();enter();ticks(125);if(state!=1||dut.catch_armed)$fatal(1,"Catch timeout/rearm hysteresis");
        restart();cfg[9*16+6]=0;angle=1290;ticks(20);angle=1330;ticks(20);if(state==5)$fatal(1,"Disabled catch entered");
        restart();cfg[9*16]=0;angle=1290;ticks(20);angle=1330;ticks(20);if(state==5)$fatal(1,"Balance disabled but catch entered");
        $display("LAB3_R1_CATCH_SUPERVISOR_PASS entry warm_D no_fast_capture strict_handoff stop estop ADC encoder both_limits timeout optional");$finish;
    end
    initial begin #2000000;$fatal(1,"Catch supervisor timeout");end
endmodule
