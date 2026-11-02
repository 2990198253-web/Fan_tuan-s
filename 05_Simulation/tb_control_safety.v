`timescale 1ns/1ps
module tb_control_safety;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0, enable = 0, update = 0;
    reg [9:0] angle = 767;
    reg signed [31:0] offset = 0;
    wire signed [15:0] demand, reverse_demand;
    angle_pd_controller #(.UPRIGHT_ADC(767), .ANGLE_DEADBAND_ADC(0),
        .KP_DUTY_PER_COUNT(16'sd4), .KD_DUTY_PER_COUNT(16'sd8),
        .CONTROL_DIV(1000), .DERIVATIVE_REFERENCE_CYCLES(1000),
        .DERIVATIVE_FILTER_SHIFT(0), .MAX_COMMAND(100), .MAX_COMMAND_STEP(0)) pd (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_raw(angle),
        .target_offset_q8(offset),.command(demand));
    angle_pd_controller #(.UPRIGHT_ADC(767), .ANGLE_DEADBAND_ADC(0),
        .KP_DUTY_PER_COUNT(16'sd4), .KD_DUTY_PER_COUNT(16'sd8),
        .CONTROL_DIV(1000), .DERIVATIVE_REFERENCE_CYCLES(1000), .MOTOR_POLARITY(1),
        .DERIVATIVE_FILTER_SHIFT(0), .MAX_COMMAND(100), .MAX_COMMAND_STEP(0)) pd_reverse (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_raw(angle),
        .target_offset_q8(offset),.command(reverse_demand));
    wire signed [15:0] filtered_demand;
    angle_pd_controller #(.UPRIGHT_ADC(767), .ANGLE_DEADBAND_ADC(0),
        .KP_DUTY_PER_COUNT(16'sd0), .KD_DUTY_PER_COUNT(16'sd8),
        .CONTROL_DIV(1000), .DERIVATIVE_REFERENCE_CYCLES(5000),
        .DERIVATIVE_FILTER_SHIFT(2), .MAX_COMMAND(100), .MAX_COMMAND_STEP(0)) pd_filtered (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_raw(angle),
        .target_offset_q8(offset),.command(filtered_demand));
    wire signed [15:0] slew_demand;
    angle_pd_controller #(.UPRIGHT_ADC(767), .ANGLE_DEADBAND_ADC(0),
        .KP_DUTY_PER_COUNT(16'sd4), .KD_DUTY_PER_COUNT(16'sd0),
        .CONTROL_DIV(1000), .DERIVATIVE_REFERENCE_CYCLES(1000),
        .MAX_COMMAND(100), .MAX_COMMAND_STEP(10)) pd_slew (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_raw(angle),
        .target_offset_q8(offset),.command(slew_demand));

    reg drive_enable = 0;
    reg signed [15:0] pwm_command = 0;
    wire in1,in2,pwm;
    motor_signed_pwm #(.PWM_PERIOD(20),.REVERSE_BLANK_CYCLES(6)) driver (
        .clk(clk),.rst_n(rst_n),.drive_enable(drive_enable),.fault(1'b0),
        .command(pwm_command),.in1(in1),.in2(in2),.pwm(pwm));
    integer n, high_count;
    task cycles;
        input integer amount;
        integer j;
        begin for(j=0;j<amount;j=j+1) begin @(posedge clk); #1; end end
    endtask
    task pd_tick;
        begin @(negedge clk); update=1; cycles(1); @(negedge clk); update=0; cycles(5); end
    endtask
    initial begin #100000; $fatal(1,"Control test timeout"); end
    initial begin
        cycles(3); @(negedge clk); rst_n=1; cycles(3);
        angle=777; cycles(2); enable=1; pd_tick();
        if(demand!==40 || reverse_demand!==-40 || filtered_demand!==0 || slew_demand!==10)
            $fatal(1,"Arm derivative kick/polarity/P/slew");
        angle=778; pd_tick();
        if(demand!==52 || reverse_demand!==-52 || filtered_demand!==10)
            $fatal(1,"Derivative normalization/filter");
        pd_tick();
        if(demand!==44 || filtered_demand!==7)
            $fatal(1,"Derivative decay");
        offset=256; pd_tick();
        if(demand!==40) $fatal(1,"Target step incorrectly caused derivative kick");
        angle=900; pd_tick(); if(demand!==100) $fatal(1,"Positive clamp");
        angle=650; pd_tick(); if(demand!==-100) $fatal(1,"Negative clamp");
        enable=0; cycles(1); if(demand!==0 || slew_demand!==0) $fatal(1,"Disarm must bypass slew");

        drive_enable=1; pwm_command=5; cycles(3);
        high_count=0;
        for(n=0;n<20;n=n+1) begin cycles(1); if(pwm) high_count=high_count+1; end
        if(high_count!==5 || !in1 || in2) $fatal(1,"PWM duty/direction");
        @(negedge clk); pwm_command=-5; #1;
        if(in1 || in2 || pwm) $fatal(1,"Reversal inhibit not immediate");
        cycles(1);
        for(n=0;n<5;n=n+1) begin cycles(1); if(in1||in2||pwm) $fatal(1,"Reversal blank too short"); end
        cycles(3); if(in1 || !in2) $fatal(1,"Reverse did not resume");
        @(negedge clk); pwm_command=0; cycles(1);
        @(negedge clk); pwm_command=5; cycles(1);
        for(n=0;n<3;n=n+1) begin cycles(1); if(in1||in2||pwm) $fatal(1,"Zero command bypassed blank"); end
        cycles(5); if(!in1 || in2) $fatal(1,"Zero/reverse did not resume");
        drive_enable=0; #1; if(in1||in2||pwm) $fatal(1,"Disable inhibit");
        $display("V2_CONTROL_SAFETY_PASS"); $finish;
    end
endmodule
