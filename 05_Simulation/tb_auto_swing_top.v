`timescale 1ns/1ps
module tb_auto_swing_top;
    reg clk=0;always #10 clk=~clk;
    reg rst=0;reg [3:0] keys=15;reg [10:0] angle=450;
    reg signed [31:0] position=100;
    wire a,b,p,ua,ub,up,tx;
    pendulum_v6_top #(.KEY_DEBOUNCE_CYCLES(4),.CONTROL_DIV(100),
        .POSITION_DIV(300),.PWM_PERIOD(50),.ADC_SAMPLE_DIV(20),.ADC_AVERAGE_LOG2(3),
        .SWING_VELOCITY_TICKS(2),.SWING_CAPTURE_SAMPLES(2),.SWING_KICK_TICKS(3),
        .SWING_RAMP_TICKS(5),.SWING_TIMEOUT_TICKS(100),.SENSOR_TIMEOUT_CYCLES(1000),
        .SWING_SOFT_TRAVEL_COUNTS(20),.SWING_HARD_TRAVEL_COUNTS(40)) dut
        (.clk_50m(clk),.rst_n(rst),.key_sw(keys),.adc_data_in(10'd225),
         .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
         .BN1(a),.BN2(b),.PWMB(p),.AN1(ua),.AN2(ub),.PWMA(up),.uart_tx(tx));
    task cycles;input integer n;begin repeat(n)begin @(posedge clk);#1;end end endtask
    task press;input integer k;begin @(negedge clk);keys[k]=0;cycles(15);@(negedge clk);keys[k]=1;cycles(15);end endtask
    task reset_case;begin @(negedge clk);rst=0;keys=15;angle=450;position=100;cycles(4);rst=1;cycles(30);end endtask
    initial begin
        force dut.adc.angle_x2=angle;force dut.adc.angle_valid=1'b1;force dut.adc.angle_update=1'b1;
        force dut.encoder1=position;
        $dumpfile("smooth_results/v6_handoff.vcd");
        $dumpvars(1,dut);
        reset_case();cycles(1000);
        if(dut.armed||a||b||p)$fatal(1,"Uncommanded startup");
        press(0);cycles(450);
        if(!dut.swing_active||!dut.armed||dut.motor_command<=0||!dut.serial_enabled||!dut.debug_word[31])
            $fatal(1,"K1 failed automatic start from down");
        if(dut.SWING_IMPULSE_ENABLE!=1 || dut.debug_word[26:23]!=4'hE ||
            dut.debug_word[22:19]!=dut.swing_action || dut.debug_word[18:11]!=dut.swing_target_offset)
            $fatal(1,"Production trajectory or action/target telemetry not connected");
        if(dut.safe_active||dut.controller.history_ready||dut.position_offset_q8!=0||ua||ub||up)
            $fatal(1,"Balance/unused motor active during swing");
        position=110;angle=1510;cycles(1300);
        if(!dut.balance_mode||dut.swing_active||dut.debug_word[31]||dut.position_target!=110||
            dut.position_offset_q8!=0||dut.motor_command!=0||dut.fault_latched)
            $fatal(1,"Capture did not hand off and rebase outer loop at actual capture position");
        angle=1511;cycles(500);
        if(dut.motor_command<=0||dut.fault_latched)$fatal(1,"Existing balance PD not operating after handoff");
        @(negedge clk);keys[1]=0;#1;
        if(a||b||p||dut.motor_command!=0)$fatal(1,"Balance raw K2 gate");
        cycles(20);keys[1]=1;cycles(30);
        if(dut.armed)$fatal(1,"Normal stop not idle");
        angle=450;press(0);cycles(350);
        @(negedge clk);keys[3]=0;#1;
        if(a||b||p||dut.motor_command!=0)$fatal(1,"Swing raw K4 gate");
        cycles(20);keys[3]=1;press(0);if(!dut.emergency_stop||dut.armed)$fatal(1,"Estop allowed restart");
        reset_case();press(0);cycles(350);position=140;#1;
        if(a||b||p||dut.motor_command!=0)$fatal(1,"Top hard travel bypassed");
        cycles(5);if(!dut.fault_latched||dut.first_inhibit_reason!=6)$fatal(1,"Top travel fault");
        reset_case();press(0);cycles(350);
        force dut.adc.angle_update=1'b0;cycles(1200);
        if(!dut.fault_latched||dut.first_inhibit_reason!=4||a||b||p)$fatal(1,"Top sensor timeout");
        $display("V6_AUTO_SWING_TOP_PASS K1_down_to_swing handoff_outer_rebase existing_PD raw_K2_K4 travel stale_sensor telemetry_flag");
        $finish;
    end
    initial begin #5000000;$fatal(1,"Auto top timeout");end
endmodule
