`timescale 1ns/1ps
module tb_dual_integral_wiring;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [3:0] keys=4'b1111;
    reg [9:0] data=764;
    // Explicit small-KI reference; production defaults may be tuned independently.
    pendulum_v2_top #(.UPRIGHT_ADC(764),.ANGLE_DEADBAND_X2(2),.ADC_AVERAGE_LOG2(4),
        .ENABLE_POSITION_LOOP(1),.ENCODER_POLARITY(0),.POSITION_POLARITY(0),
        .DEADBAND_CONFIRM_TICKS(5),.KI_DUTY_PER_COUNT_Q8(16'sd1),.POSITION_KI_Q16(16'sd1),
        .POSITION_KP_Q8(16'sd8),.KEY_DEBOUNCE_CYCLES(4),.ADC_SAMPLE_DIV(20),.CONTROL_DIV(50),
        .POSITION_DIV(500),.SENSOR_TIMEOUT_CYCLES(2500),.KD_DUTY_PER_COUNT(16'sd0),
        .POSITION_KD_Q8(16'sd0)) dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.adc_data_in(data),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0));
    task cycles; input integer count; integer n;
        begin for(n=0;n<count;n=n+1) begin @(posedge clk); #1; end end endtask
    initial begin #1000000; $fatal(1,"Dual integral wiring timeout"); end
    initial begin
        force dut.encoder1=32'sd100;
        cycles(3); rst_n=1; cycles(360);
        keys[0]=0; cycles(12); keys[0]=1; cycles(400);
        if(!dut.armed || dut.KI_DUTY_PER_COUNT_Q8!==1 || dut.POSITION_KI_Q16!==1 ||
           dut.controller.integral_q16!==0 || dut.position_controller.integral_q16!==0)
            $fatal(1,"Reference dual-KI configuration or center state incorrect");
        force dut.encoder1=32'sd108; cycles(6400);
        if(dut.controller.integral_q16>=0 || dut.position_controller.integral_q16>=0 ||
           dut.position_offset_q8<=0 || dut.pd_command>=0 || dut.motor_command>=0 || dut.fault_latched)
            $fatal(1,"One integral was not wired or used the wrong restoring sign");
        keys[1]=0; cycles(12); keys[1]=1; cycles(20);
        if(dut.armed || dut.controller.integral_q16!==0 || dut.position_controller.integral_q16!==0 || dut.motor_command!==0)
            $fatal(1,"K2 did not clear both integral states");
        release dut.encoder1;
        $display("V2_DUAL_INTEGRAL_WIRING_PASS"); $finish;
    end
endmodule
