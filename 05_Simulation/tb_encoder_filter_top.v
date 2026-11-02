`timescale 1ns/1ps
module tb_encoder_filter_top;
    reg clk=0;always #10 clk=~clk;
    reg rst=0;reg [3:0] keys=15;reg [1:0] ab=0;
    wire a1,a2,ap,b1,b2,bp;
    pendulum_v6_top #(.KEY_DEBOUNCE_CYCLES(4),.CONTROL_DIV(50),.POSITION_DIV(200),.SWING_VELOCITY_TICKS(2)) dut(
        .clk_50m(clk),.rst_n(rst),.key_sw(keys),.adc_data_in(10'd220),
        .enc1_a(ab[1]),.enc1_b(ab[0]),.enc2_a(1'b0),.enc2_b(1'b0),
        .AN1(a1),.AN2(a2),.PWMA(ap),.BN1(b1),.BN2(b2),.PWMB(bp));
    task cycles;input integer n;begin repeat(n)@(negedge clk);end endtask
    initial begin
        force dut.adc.angle_x2=11'd440;force dut.adc.angle_valid=1'b1;force dut.adc.angle_update=1'b1;
        cycles(4);rst=1;cycles(120);keys[0]=0;cycles(15);keys[0]=1;cycles(120);
        if(dut.ENCODER_FILTER_CYCLES!=50||!dut.swing_active)$fatal(1,"Production filter not wired/start failed");
        ab=3;cycles(2);ab=0;cycles(100);
        if(dut.encoder1_illegal||dut.fault_latched||dut.motor_command==0)$fatal(1,"Brief spike stopped production top");
        ab=3;cycles(70);
        if(!dut.encoder1_illegal||!dut.fault_latched||dut.first_inhibit_reason!=7||dut.motor_command!=0||a1||a2||ap||b1||b2||bp)
            $fatal(1,"Persistent invalid encoder must latch code7 and shut down both ports");
        $display("V6R2_ENCODER_FILTER_TOP_PASS brief_spike_continues persistent_error_code7_outputs_zero");
        $finish;
    end
    initial begin #200000;$fatal(1,"Filtered top timeout");end
endmodule
