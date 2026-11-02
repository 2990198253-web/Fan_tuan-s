`timescale 1ns/1ps
module tb_auto_motor_ports;
    reg clk=0;always #10 clk=~clk;
    reg rst=0;reg [3:0] keys=15;reg [10:0] angle=450;
    reg signed [31:0] one=100,two=-100;
    wire a1,a2,ap,b1,b2,bp,ua1,ua2,uap,ub1,ub2,ubp;
    pendulum_v6_top #(.KEY_DEBOUNCE_CYCLES(4),.CONTROL_DIV(50),.POSITION_DIV(200),
        .KD_DUTY_PER_COUNT(16'sd0),.PWM_PERIOD(50),.SWING_VELOCITY_TICKS(2),
        .SWING_CAPTURE_SAMPLES(2),.SWING_TIMEOUT_TICKS(1000),.MOTOR_PORT(1)) port1
        (.clk_50m(clk),.rst_n(rst),.key_sw(keys),.adc_data_in(10'd225),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .AN1(ua1),.AN2(ua2),.PWMA(uap),.BN1(b1),.BN2(b2),.PWMB(bp));
    pendulum_v6_top #(.KEY_DEBOUNCE_CYCLES(4),.CONTROL_DIV(50),.POSITION_DIV(200),
        .KD_DUTY_PER_COUNT(16'sd0),.PWM_PERIOD(50),.SWING_VELOCITY_TICKS(2),
        .SWING_CAPTURE_SAMPLES(2),.SWING_TIMEOUT_TICKS(1000),.MOTOR_PORT(2),
        .ENCODER_POLARITY(1),.MOTOR_POLARITY(1)) port2
        (.clk_50m(clk),.rst_n(rst),.key_sw(keys),.adc_data_in(10'd225),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .AN1(a1),.AN2(a2),.PWMA(ap),.BN1(ub1),.BN2(ub2),.PWMB(ubp));
    task cycles;input integer n;begin repeat(n)begin @(posedge clk);#1;end end endtask
    initial begin
        force port1.adc.angle_x2=angle;force port2.adc.angle_x2=angle;
        force port1.adc.angle_valid=1'b1;force port2.adc.angle_valid=1'b1;
        force port1.adc.angle_update=1'b1;force port2.adc.angle_update=1'b1;
        force port1.encoder1=one;force port1.encoder2=32'sd999999;
        force port2.encoder2=two;force port2.encoder1=-32'sd999999;
        force port1.encoder2_illegal=1'b1;force port2.encoder1_illegal=1'b1;
        cycles(4);rst=1;cycles(30);keys[0]=0;cycles(15);keys[0]=1;cycles(400);
        if(!port1.swing_active||!port2.swing_active||port1.motor_command<=0||port2.motor_command>=0||
            ua1||ua2||uap||ub1||ub2||ubp||b1||!b2||a1||!a2)
            $fatal(1,"Port mapping or swing polarity or unused encoder gating");
        one=108;two=-108;angle=1510;cycles(700);
        if(!port1.balance_mode||!port2.balance_mode||port1.position_target!=108||port2.position_target!=108||
            port1.fault_latched||port2.fault_latched)
            $fatal(1,"Selected/normalized encoder not recaptured on both ports");
        angle=1511;cycles(400);
        if(port1.motor_command!==110||port2.motor_command!==-110||b1||!b2||a1||!a2)
            $fatal(1,"Balance phase port/polarity mismatch");
        @(negedge clk);keys[1]=0;#1;
        if(a1||a2||ap||b1||b2||bp)$fatal(1,"Both ports raw stop");
        $display("V6_AUTO_MOTOR_PORTS_PASS B_port1 A_port2 reversed_motor_and_encoder unused_encoder_safe capture_and_PD");
        $finish;
    end
    initial begin #3000000;$fatal(1,"Motor ports timeout");end
endmodule
