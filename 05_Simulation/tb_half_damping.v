`timescale 1ns/1ps
module tb_half_damping;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, enable=0, update=0;
    reg [10:0] angle=1527;
    reg signed [31:0] offset=0;
    wire signed [15:0] demand;
    wire confirmed;
    balance_pd_controller #(.KP(16'sd0),.KD(16'sd4),.CONTROL_DIV(1000),
        .DERIVATIVE_WINDOW_TICKS(4),.DERIVATIVE_FILTER_SHIFT(0)) d (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(demand),.deadband_confirmed(confirmed));
    task cycles; input integer count; integer n;
        begin for(n=0;n<count;n=n+1) begin @(posedge clk); #1; end end endtask
    task tick;
        begin @(negedge clk); update=1; cycles(1); @(negedge clk); update=0; cycles(4); end endtask
    initial begin #100000; $fatal(1,"Half damping timeout"); end
    initial begin
        cycles(3); rst_n=1; cycles(3); enable=1; tick();
        angle=1528; tick(); if(demand!==125) $fatal(1,"Half-count derivative scaling");
        tick(); tick(); tick();
        if(!confirmed || demand!==125) $fatal(1,"Center qualification erased required damping");
        tick(); if(demand!==0) $fatal(1,"Stopped measurement retained false velocity");
        offset=256; tick(); if(demand!==0) $fatal(1,"Outer target introduced derivative kick");
        angle=1527; tick(); if(demand!==-125) $fatal(1,"Negative half-count derivative asymmetry");
        enable=0; angle=2000; cycles(2); enable=1; tick();
        if(demand!==0) $fatal(1,"Arm introduced velocity history kick");
        $display("V2_HALF_DAMPING_PASS"); $finish;
    end
endmodule
