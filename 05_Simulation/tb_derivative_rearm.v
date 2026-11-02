`timescale 1ns/1ps
module tb_derivative_rearm;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,en=0,update=0;
    reg [10:0] angle=2000;
    wire signed [15:0] command;
    balance_pd_controller #(.UPRIGHT_ADC_X2(1528),.ANGLE_DEADBAND_X2(0),.KP(140),.KD(300),
        .MAX_D_COMMAND(1800),.DERIVATIVE_WINDOW_TICKS(5),
        .DERIVATIVE_FILTER_SHIFT(1),.CONTROL_DIV(25000)) dut (
        .clk(clk),.rst_n(rst_n),.enable(en),.update(update),.angle_x2(angle),
        .target_offset_q8(32'sd0),.command(command));
    task tick;
        begin @(negedge clk);update=1;@(negedge clk);update=0;repeat(5)@(negedge clk);end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;repeat(3)@(negedge clk);
        // A sensor transition can make enable and the first valid measurement
        // change together. Data observed while inhibited is not a velocity sample.
        angle=1528;en=1;tick;
        if(command!==0)$fatal(1,"Rearm generated false D from inhibited measurement: %0d",command);
        repeat(6)begin tick;if(command!==0)$fatal(1,"Old history survived rearm");end
        angle=1529;tick;
        if(command!==220)$fatal(1,"Real velocity after rearm was suppressed: %0d",command);
        en=0;angle=0;repeat(3)@(negedge clk);angle=1528;en=1;tick;
        if(command!==0)$fatal(1,"Negative inhibited measurement caused false D");
        en=0;repeat(3)@(negedge clk);angle=1536;en=1;tick;
        if(command!==560)$fatal(1,"Rearm must preserve immediate P correction");
        $display("V2_DERIVATIVE_REARM_PASS");$finish;
    end
endmodule
