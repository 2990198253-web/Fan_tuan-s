`timescale 1ns/1ps
module tb_inner_only_guard_off;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,enable=0,capture=0,update=0;
    reg signed [31:0] position=1000;
    reg [15:0] kp=0,ki=0,kd=0;
    wire signed [31:0] offset,target,anchor,velocity;
    wire signed [32:0] error;
    position_pid_controller #(.RUNTIME_GAINS(1),.OUTPUT_POLARITY(1),
        .MAX_OFFSET_ADC(8),.MAX_OFFSET_STEP_Q8(32),
        .SOFT_BOUND_COUNTS(10),.SOFT_BOUND_MARGIN_COUNTS(0),
        .SOFT_BOUND_GAIN_Q8(32),.SOFT_CLIP_KNEE_ADC(4),
        .DISABLE_GUARD_WITH_ZERO_GAINS(1)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.capture(capture),.update(update),
        .runtime_kp_q8(kp),.runtime_ki_q16(ki),.runtime_kd_q8(kd),
        .reset_integral(1'b0),.move_request(1'b0),
        .move_relative(16'sd0),.move_speed(16'd10),.position(position),
        .target_offset_q8(offset),.target_position(target),.anchor_position(anchor),
        .position_error(error),.velocity_q8(velocity));
    task tick;begin @(negedge clk);update=1;@(negedge clk);update=0;
        repeat(6)@(negedge clk);end endtask
    task ticks;input integer n;integer j;begin for(j=0;j<n;j=j+1)tick();end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;enable=1;
        @(negedge clk);capture=1;@(negedge clk);capture=0;
        position=1030;ticks(50);
        if(offset!==0)$fatal(1,"all-zero outer gains still moved the angle target");
        kp=17;ticks(50);
        if(offset!==-1087)$fatal(1,"restored outer P/guard correction mismatch: %0d",offset);
        kp=0;ticks(50);
        if(offset!==0)$fatal(1,"zeroing outer gains did not release the guard");
        $display("INNER_ONLY_GUARD_OFF_PASS zero_outer disables_guard restores_guard");
        $finish;
    end
    initial begin #1000000;$fatal(1,"guard-off timeout");end
endmodule
