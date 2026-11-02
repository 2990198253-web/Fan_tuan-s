`timescale 1ns/1ps
module tb_inner_live_pid;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, enable=0, update=0, reset_integral=0;
    reg [10:0] angle=1527;
    reg [15:0] kp=220,ki=0,kd=0;
    wire signed [15:0] command;
    balance_pd_controller #(.RUNTIME_GAINS(1),.ANGLE_DEADBAND_X2(0),
        .INTEGRAL_LIMIT_COMMAND(100),.INTEGRAL_WINDOW_ADC(10)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.reset_integral(reset_integral),
        .runtime_kp(kp),.runtime_ki_q8(ki),.runtime_kd(kd),
        .angle_x2(angle),.upright_adc_x2(11'd1527),.target_offset_q8(32'sd0),.command(command));
    task cycles;input integer n;integer j;begin for(j=0;j<n;j=j+1)begin @(posedge clk);#1;end end endtask
    task tick;begin @(negedge clk);update=1;cycles(1);@(negedge clk);update=0;cycles(4);end endtask
    initial begin #200000;$fatal(1,"inner live timeout");end
    initial begin
        cycles(3);rst_n=1;cycles(2);enable=1;angle=1529;
        tick();if(command!==220)$fatal(1,"default live P mismatch %d",command);
        kp=400;tick();if(command!==400)$fatal(1,"P change failed %d",command);
        kp=0;ki=64;tick();tick();
        if(dut.integral_q16<=0)$fatal(1,"I did not accumulate");
        @(negedge clk);reset_integral=1;cycles(1);@(negedge clk);reset_integral=0;
        if(dut.integral_q16!==0)$fatal(1,"gain change did not clear I");
        ki=0;kd=100;angle=1531;tick();
        if(command<=0)$fatal(1,"D change did not respond");
        enable=0;cycles(1);if(command!==0)$fatal(1,"disable failed");
        $display("R5_INNER_LIVE_PID_PASS");$finish;
    end
endmodule
