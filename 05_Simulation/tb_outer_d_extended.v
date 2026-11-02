`timescale 1ns/1ps
module tb_outer_d_extended;
    reg clk=0; always #10 clk=~clk;
    reg rst=0,enable=0,capture=0,update=0;
    reg signed [31:0] position=0;
    wire signed [31:0] high_offset,old_offset;
    position_pid_controller #(.RUNTIME_GAINS(1),.MAX_OFFSET_ADC(100),.MAX_OFFSET_STEP_Q8(0)) high_d (
        .clk(clk),.rst_n(rst),.enable(enable),.capture(capture),.update(update),
        .runtime_kp_q8(16'd0),.runtime_ki_q16(16'd0),.runtime_kd_q8(16'd8191),
        .reset_integral(1'b0),.move_request(1'b0),.move_relative(16'sd0),.move_speed(16'd0),
        .position(position),.target_offset_q8(high_offset));
    position_pid_controller #(.RUNTIME_GAINS(1),.MAX_OFFSET_ADC(100),.MAX_OFFSET_STEP_Q8(0)) old_d (
        .clk(clk),.rst_n(rst),.enable(enable),.capture(capture),.update(update),
        .runtime_kp_q8(16'd0),.runtime_ki_q16(16'd0),.runtime_kd_q8(16'd2048),
        .reset_integral(1'b0),.move_request(1'b0),.move_relative(16'sd0),.move_speed(16'd0),
        .position(position),.target_offset_q8(old_offset));
    task tick; begin @(negedge clk); update=1; @(negedge clk); update=0;
        repeat(7) @(negedge clk); end endtask
    initial begin
        repeat(3) @(negedge clk); rst=1; enable=1;
        @(negedge clk);capture=1;@(negedge clk);capture=0;
        position=1;tick();tick();
        if(high_d.active_kd_q8!==8191 || high_offset<=old_offset || old_offset<=0)
            $fatal(1,"Extended D was truncated or did not increase braking: %0d %0d",high_offset,old_offset);
        $display("OUTER_D_EXTENDED_PASS 8191_applied stronger_than_2048"); $finish;
    end
    initial begin #100000; $fatal(1,"Extended D timeout"); end
endmodule
