`timescale 1ns/1ps
module tb_half_integral;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, enable=0, update=0;
    reg [10:0] angle=1527;
    reg signed [31:0] offset=0;
    wire signed [15:0] half_i, capped_i, anti_i, reversed_anti;
    balance_pd_controller #(.ANGLE_DEADBAND_X2(0),.KP(16'sd0),.KD(16'sd0),.KI_Q8(16'sd1),
        .INTEGRAL_LIMIT_COMMAND(3),.INTEGRAL_WINDOW_ADC(10)) h (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(half_i));
    balance_pd_controller #(.ANGLE_DEADBAND_X2(0),.KP(16'sd0),.KD(16'sd0),.KI_Q8(16'sd256),
        .INTEGRAL_LIMIT_COMMAND(3),.INTEGRAL_WINDOW_ADC(10)) cap (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(capped_i));
    balance_pd_controller #(.ANGLE_DEADBAND_X2(0),.KP(16'sd1),.KD(16'sd0),.KI_Q8(16'sd256),
        .INTEGRAL_LIMIT_COMMAND(20),.INTEGRAL_WINDOW_ADC(0),.MAX_COMMAND(3)) anti (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(anti_i));
    balance_pd_controller #(.ANGLE_DEADBAND_X2(0),.KP(16'sd1),.KD(16'sd0),.KI_Q8(16'sd256),
        .INTEGRAL_LIMIT_COMMAND(20),.INTEGRAL_WINDOW_ADC(0),.MAX_COMMAND(3),.MOTOR_POLARITY(1)) reverse (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(reversed_anti));
    integer n;
    task cycles; input integer count; integer j;
        begin for(j=0;j<count;j=j+1) begin @(posedge clk); #1; end end endtask
    task tick;
        begin @(negedge clk); update=1; cycles(1); @(negedge clk); update=0; cycles(4); end endtask
    task clear;
        begin enable=0; angle=1527; offset=0; cycles(2); enable=1; end endtask
    initial begin #2000000; $fatal(1,"Half integral timeout"); end
    initial begin
        cycles(3); rst_n=1; cycles(3); enable=1; angle=1528;
        tick(); if(h.integral_q16!==128) $fatal(1,"Half-count KI increment was truncated away");
        for(n=0;n<511;n=n+1) tick();
        if(half_i!==1 || h.integral_q16!==65536 || capped_i!==3) $fatal(1,"Positive half-count integration or clamp");
        angle=1527; for(n=0;n<6;n=n+1) tick();
        if(half_i!==1 || h.integral_q16!==65536) $fatal(1,"Center erased useful accumulated I");
        clear(); angle=1526; for(n=0;n<512;n=n+1) tick();
        if(half_i!==-1 || h.integral_q16!==-65536 || capped_i!==-3) $fatal(1,"Negative half-count integration symmetry");
        clear(); offset=64; for(n=0;n<1024;n=n+1) tick();
        if(half_i!==-1 || h.integral_q16!==-65536) $fatal(1,"Fractional outer target failed to integrate");
        angle=1567; offset=0; tick();
        if(half_i!==0 || h.integral_q16!==0) $fatal(1,"Integral separation did not clear large-error residue");
        clear(); angle=1547; for(n=0;n<10;n=n+1) tick();
        if(anti_i!==3 || reversed_anti!==-3 || anti.integral_q16!==0 || reverse.integral_q16!==0)
            $fatal(1,"Saturated controller accumulated I or polarity changed anti-windup");
        angle=1527; tick(); if(anti_i!==0 || reversed_anti!==0) $fatal(1,"Saturation left an integral bias");
        enable=0; cycles(1); if(half_i!==0 || capped_i!==0 || h.integral_q16!==0)
            $fatal(1,"Disable/reset retained integral force");
        $display("V2_HALF_INTEGRAL_PASS"); $finish;
    end
endmodule
