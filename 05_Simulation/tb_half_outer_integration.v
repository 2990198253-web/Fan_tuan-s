`timescale 1ns/1ps
module tb_half_outer_integration;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [3:0] keys=4'b1111;
    reg [9:0] data=764;
    wire b1,b2,bp,a1,a2,ap;
    // Fixed historical PD reference, independent of user-tuned production defaults.
    // This regression intentionally checks the legacy +/-600 minimum drive.
    pendulum_v2_top #(.SMOOTH_BALANCE_ENABLE(0),.UPRIGHT_ADC(764),.ANGLE_DEADBAND_X2(2),.ADC_AVERAGE_LOG2(4),
        .ENABLE_POSITION_LOOP(1),.ENCODER_POLARITY(0),.POSITION_POLARITY(0),
        .DEADBAND_CONFIRM_TICKS(5),.KP_DUTY_PER_COUNT(16'sd250),.POSITION_KP_Q8(16'sd8),.MIN_ACTIVE_COMMAND(600),
        .KEY_DEBOUNCE_CYCLES(4),.ADC_SAMPLE_DIV(20),.CONTROL_DIV(50),
        .POSITION_DIV(500),.SENSOR_TIMEOUT_CYCLES(2500),.POSITION_KD_Q8(16'sd0),.POSITION_KI_Q16(16'sd0),
        .KD_DUTY_PER_COUNT(16'sd0),.KI_DUTY_PER_COUNT_Q8(16'sd0)) dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.adc_data_in(data),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .BN1(b1),.BN2(b2),.PWMB(bp));
    pendulum_v2_top #(.SMOOTH_BALANCE_ENABLE(0),.UPRIGHT_ADC(764),.ANGLE_DEADBAND_X2(2),.ADC_AVERAGE_LOG2(4),.ENABLE_POSITION_LOOP(1),
        .DEADBAND_CONFIRM_TICKS(5),.KP_DUTY_PER_COUNT(16'sd250),.POSITION_KP_Q8(16'sd8),.MIN_ACTIVE_COMMAND(600),
        .KEY_DEBOUNCE_CYCLES(4),.ADC_SAMPLE_DIV(20),.CONTROL_DIV(50),
        .POSITION_DIV(500),.SENSOR_TIMEOUT_CYCLES(2500),.POSITION_KD_Q8(16'sd0),.POSITION_KI_Q16(16'sd0),
        .MOTOR_PORT(2),.MOTOR_POLARITY(1),.ENCODER_POLARITY(1),.POSITION_POLARITY(0),
        .KD_DUTY_PER_COUNT(16'sd0),.KI_DUTY_PER_COUNT_Q8(16'sd0)) port2 (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.adc_data_in(data),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .AN1(a1),.AN2(a2),.PWMA(ap));
    task cycles; input integer count; integer n;
        begin for(n=0;n<count;n=n+1) begin @(posedge clk); #1; end end endtask
    task press; input integer number;
        begin @(negedge clk); keys[number]=0; cycles(12); @(negedge clk); keys[number]=1; cycles(12); end endtask
    task reset_arm;
        begin @(negedge clk); rst_n=0; keys=4'b1111; cycles(3); rst_n=1; cycles(360); press(0); cycles(400); end endtask
    initial begin #10000000; $fatal(1,"Half ADC/outer integration timeout"); end
    initial begin
        force dut.encoder1=32'sd100; force port2.encoder2=-32'sd100;
        reset_arm();
        if(!dut.armed || dut.angle_x2!==1528 || dut.angle_raw!==764 || !dut.deadband_confirmed ||
           dut.pd_command!==0 || dut.motor_command!==0 || dut.position_target!==100)
            $fatal(1,"Fixed ADC 764 center, qualification, or reference capture failed");
        force dut.encoder1=32'sd108; force port2.encoder2=-32'sd108; cycles(16000);
        if(dut.position_offset_q8!==64 || port2.position_offset_q8!==-64 || dut.pd_command!==-62 ||
           dut.motor_command!==-600 || port2.motor_command!==-600 || b1 || !b2 || a1 || !a2 || dut.fault_latched)
            $fatal(1,"Positive displacement failed restoring action despite center deadband");
        if(dut.debug_word[10:0]!==1528 || $signed(dut.debug_word[26:11])!==64 || dut.first_inhibit_reason!==0)
            $fatal(1,"Live half ADC/outer diagnostic word mismatch");
        if(dut.telemetry.encoder1!==108 || dut.telemetry.encoder2!==$signed(dut.debug_word))
            $fatal(1,"Auxiliary view overwrote active encoder");
        press(0); if(dut.position_target!==100) $fatal(1,"Running K1 moved fixed reference");
        force dut.encoder1=32'sd92; force port2.encoder2=-32'sd92; cycles(16000);
        if(dut.position_offset_q8!==-64 || dut.pd_command!==62 || dut.motor_command!==600 ||
           !b1 || b2 || !a1 || a2 || dut.position_target!==100)
            $fatal(1,"Negative displacement failed restoring action");
        keys[1]=0; #1; if(b1||b2||bp||a1||a2||ap) $fatal(1,"Window delayed K2 inhibit");
        cycles(12); keys[1]=1; cycles(12);
        if(dut.armed || dut.first_inhibit_reason!==1 || dut.position_offset_q8!==0)
            $fatal(1,"K2 latch/diagnostic reset");
        reset_arm(); force dut.adc.angle_x2=11'd1929; #1;
        if(b1||b2||bp||dut.motor_command!==0) $fatal(1,"Half-count safety boundary not immediate");
        cycles(1000); if(!dut.fault_latched || dut.first_inhibit_reason!==3) $fatal(1,"Angle fault record");
        release dut.adc.angle_x2;
        reset_arm(); force dut.sensor_age=32'd2500; cycles(3);
        if(!dut.fault_latched || dut.first_inhibit_reason!==4 || dut.motor_command!==0) $fatal(1,"Sensor safety record");
        release dut.sensor_age;
        reset_arm(); press(3);
        if(!dut.emergency_stop || dut.motor_command!==0 || dut.first_inhibit_reason!==2) $fatal(1,"Emergency safety record");
        release dut.encoder1; release port2.encoder2;
        $display("V2_FIXED_764_OUTER_INTEGRATION_PASS"); $finish;
    end
endmodule
