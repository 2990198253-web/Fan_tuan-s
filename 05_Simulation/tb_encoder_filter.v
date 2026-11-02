`timescale 1ns/1ps
module tb_encoder_filter;
    reg clk=0;always #10 clk=~clk;
    reg rst=0;reg [1:0] ab=3;
    wire signed [31:0] position;wire illegal;
    quadrature_counter #(.FILTER_CYCLES(50)) dut(.clk(clk),.rst_n(rst),
        .phase_a(ab[1]),.phase_b(ab[0]),.position(position),.illegal_seen(illegal));
    task cycles;input integer n;begin repeat(n)@(negedge clk);end endtask
    task state;input [1:0] value;begin ab=value;cycles(65);end endtask
    integer i;
    initial begin
        $dumpfile("smooth_results/v6r2_encoder_filter.vcd");
        $dumpvars(0,ab,position,illegal,dut.current,dut.stable_count);
        cycles(4);rst=1;cycles(100);
        if(position!==0||illegal)$fatal(1,"Initial high AB must establish zero, not count/fault");
        state(1);state(0);state(2);state(3);
        if(position!==4||illegal)$fatal(1,"Forward filtered quadrature");
        state(2);state(0);state(1);state(3);
        if(position!==0||illegal)$fatal(1,"Reverse filtered quadrature");
        state(2);state(0);
        for(i=1;i<50;i=i+1)begin ab=3;cycles(i);ab=0;cycles(65);end
        if(position!==-2||illegal)$fatal(1,"Sub-1us AB spikes must not count or fault");
        state(2);state(3);state(1);state(0);
        if(position!==2||illegal)$fatal(1,"Counting must recover after noise");
        state(3);
        if(!illegal||position!==2)$fatal(1,"Persistent illegal 00-to-11 must still latch");
        state(1);state(0);if(!illegal)$fatal(1,"Fault must persist until reset");
        rst=0;cycles(4);rst=1;cycles(100);
        if(illegal||position!==0)$fatal(1,"Reset must clear fault and reference");
        $display("V6R2_ENCODER_FILTER_PASS forward reverse startup spikes_1_to_49_cycles persistent_illegal_latch reset");
        $finish;
    end
    initial begin #500000;$fatal(1,"Encoder filter timeout");end
endmodule
