`timescale 1ns/1ps
module tb_hard_travel_disabled;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,start=0,stop=0,estop=0;
    reg signed [31:0] position=0;
    wire disabled_motion_safe,enabled_motion_safe;
    wire [2:0] disabled_state,enabled_state;
    wire [3:0] disabled_reason,enabled_reason;
    swing_up_supervisor #(.HARD_TRAVEL_COUNTS(448),.ENABLE_HARD_TRAVEL_STOP(0)) disabled(
        .clk(clk),.rst_n(rst_n),.update(1'b0),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(11'd450),
        .upright_adc(10'd775),.reference_update_ms(1'b0),
        .lab_config_words(224'd0),.position(position),
        .motion_safe(disabled_motion_safe),.state(disabled_state),.reason(disabled_reason));
    swing_up_supervisor #(.HARD_TRAVEL_COUNTS(448),.ENABLE_HARD_TRAVEL_STOP(1)) enabled(
        .clk(clk),.rst_n(rst_n),.update(1'b0),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(11'd450),
        .upright_adc(10'd775),.reference_update_ms(1'b0),
        .lab_config_words(224'd0),.position(position),
        .motion_safe(enabled_motion_safe),.state(enabled_state),.reason(enabled_reason));
    swing_lab_top board(.clk_50m(clk),.rst_n(rst_n),.uart_rx(1'b1),
        .key_sw(4'b1111),.adc_data_in(10'd0),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0));
    task cycles;input integer n;begin repeat(n)@(negedge clk);end endtask
    initial begin
        cycles(4);rst_n=1;cycles(3);
        if(board.core.ENABLE_HARD_TRAVEL_STOP!==0 ||
           board.core.swing_controller.ENABLE_HARD_TRAVEL_STOP!==0)
            $fatal(1,"actual J280 top still enables hard travel stop");
        start=1;cycles(1);start=0;cycles(2);
        if(disabled_state!==1 || enabled_state!==1)$fatal(1,"swing did not start");
        position=32'sd500;#1;
        if(disabled_motion_safe!==1 || enabled_motion_safe!==0)
            $fatal(1,"travel guard selection failed");
        cycles(2);
        if(disabled_state!==1 || enabled_state!==3 || enabled_reason!==6)
            $fatal(1,"travel stop still fired in disabled build");
        stop=1;cycles(1);stop=0;cycles(1);
        if(disabled_state!==0)$fatal(1,"K2 stop no longer works");
        estop=1;cycles(1);
        if(disabled_state!==4 || disabled_reason!==2)$fatal(1,"K4 estop no longer works");
        $display("HARD_TRAVEL_DISABLED_PASS unbounded_position original_guard K2 K4");
        $finish;
    end
    initial begin #1000000;$fatal(1,"hard travel test timeout");end
endmodule
