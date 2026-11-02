`timescale 1ns/1ps
module tb_outer_integral;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, enable=0, capture=0, update=0;
    reg signed [31:0] position=0;
    wire signed [31:0] output_offset, flipped_offset;
    position_pid_controller #(.KP_Q8(16'sd0),.KD_Q8(16'sd0),.KI_Q16(16'sd1),
        .I_LIMIT_ADC(1),.MAX_OFFSET_ADC(3),.MAX_OFFSET_STEP_Q8(0),.INTEGRAL_WINDOW_COUNTS(64)) i (
        .clk(clk),.rst_n(rst_n),.enable(enable),.capture(capture),.update(update),.position(position),
        .target_offset_q8(output_offset));
    position_pid_controller #(.KP_Q8(16'sd0),.KD_Q8(16'sd0),.KI_Q16(16'sd1),.OUTPUT_POLARITY(1),
        .I_LIMIT_ADC(1),.MAX_OFFSET_ADC(3),.MAX_OFFSET_STEP_Q8(0),.INTEGRAL_WINDOW_COUNTS(64)) flip (
        .clk(clk),.rst_n(rst_n),.enable(enable),.capture(capture),.update(update),.position(position),
        .target_offset_q8(flipped_offset));
    integer n;
    task cycles; input integer count; integer j;
        begin for(j=0;j<count;j=j+1) begin @(posedge clk); #1; end end endtask
    task tick;
        begin @(negedge clk); update=1; cycles(1); @(negedge clk); update=0; cycles(5); end endtask
    task clear;
        begin enable=0; position=0; cycles(2); capture=1; cycles(1); capture=0; cycles(1); enable=1; end endtask
    initial begin #4000000; $fatal(1,"Outer integral timeout"); end
    initial begin
        cycles(3); rst_n=1; cycles(3); clear(); position=-8;
        tick(); if(i.integral_q16!==8) $fatal(1,"Small position KI increment was lost");
        for(n=0;n<8191;n=n+1) tick();
        if(i.integral_q16!==65536 || output_offset!==-256 || flipped_offset!==256)
            $fatal(1,"Outer KI scale/sign/limit");
        for(n=0;n<10;n=n+1) tick(); if(i.integral_q16!==65536) $fatal(1,"Outer I limit exceeded");
        position=0; tick(); if(i.integral_q16!==65536) $fatal(1,"Zero error erased steady compensation");
        position=-100; tick(); if(i.integral_q16!==0 || output_offset!==0) $fatal(1,"Outer integral separation failed");
        clear(); position=8; for(n=0;n<8192;n=n+1) tick();
        if(i.integral_q16!==-65536 || output_offset!==256) $fatal(1,"Negative outer integral symmetry");
        enable=0; cycles(1); if(i.integral_q16!==0 || output_offset!==0) $fatal(1,"Outer disable retained I");
        $display("V2_OUTER_INTEGRAL_PASS"); $finish;
    end
endmodule
