`timescale 1ns/1ps
module tb_balance_window5;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, enable=0, update=0;
    reg [10:0] angle=1527;
    reg signed [31:0] offset=0;
    wire signed [15:0] request;
    wire inside, confirmed;
    balance_pd_controller #(.KD(16'sd0)) pd (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(update),.angle_x2(angle),
        .target_offset_q8(offset),.command(request),.inside_deadband(inside),.deadband_confirmed(confirmed));
    reg out_update=0, out_inside=0, out_confirmed=0, position_action=0;
    reg signed [15:0] out_request=600;
    wire signed [15:0] output_command;
    wire signed [15:0] peak_command;
    balance_pwm_limiter #(.START_COMMAND(0)) output_stage (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(out_update),.request(out_request),
        .inside_deadband(out_inside),.deadband_confirmed(out_confirmed),
        .position_needs_action(position_action),.command(output_command));
    balance_pwm_limiter peak_output (
        .clk(clk),.rst_n(rst_n),.enable(enable),.update(out_update),.request(out_request),
        .inside_deadband(out_inside),.deadband_confirmed(out_confirmed),
        .position_needs_action(position_action),.command(peak_command));
    integer n;
    task cycles; input integer count; integer j;
        begin for(j=0;j<count;j=j+1) begin @(posedge clk); #1; end end endtask
    task tick;
        begin @(negedge clk); update=1; cycles(1); @(negedge clk); update=0; cycles(4); end endtask
    task out_tick;
        begin @(negedge clk); out_update=1; cycles(1); @(negedge clk); out_update=0; cycles(2); end endtask
    initial begin #100000; $fatal(1,"Five-sample window timeout"); end
    initial begin
        cycles(3); rst_n=1; cycles(3); enable=1; angle=1528;
        for(n=0;n<4;n=n+1) begin tick(); if(request!==125 || confirmed || !inside) $fatal(1,"P suppressed before fifth center sample"); end
        tick(); if(request!==0 || !confirmed) $fatal(1,"Fifth center sample not qualified");
        offset=64; tick(); if(request!==-62) $fatal(1,"Angle deadband killed outer-loop correction");
        offset=-64; tick(); if(request!==62) $fatal(1,"Fractional target truncation asymmetry");
        offset=0; angle=1529; tick(); if(request!==250 || confirmed || inside) $fatal(1,"Leaving deadband did not respond immediately");
        angle=1527; tick(); tick(); angle=1529; tick(); angle=1527;
        for(n=0;n<4;n=n+1) begin tick(); if(confirmed) $fatal(1,"Interrupted center visits accumulated"); end
        tick(); if(!confirmed) $fatal(1,"New continuous five samples failed");
        out_tick(); if(output_command!==600 || peak_command!==1200) $fatal(1,"Output/start setup");
        out_inside=1; out_request=0;
        for(n=0;n<4;n=n+1) begin out_tick(); if(output_command!==600 || peak_command!==1200) $fatal(1,"Zero request unloaded before qualification"); end
        out_confirmed=1; out_tick(); if(output_command!==500 || peak_command!==1100) $fatal(1,"Qualified release did not begin");
        out_confirmed=0; out_request=-100; out_tick();
        if(output_command!==-600 || peak_command>=0) $fatal(1,"Window held old force against reversal");
        out_confirmed=1; position_action=1; out_request=100; out_tick();
        if(output_command!==600) $fatal(1,"Center qualification suppressed position actuation");
        enable=0; cycles(1); if(output_command!==0 || peak_command!==0 || request!==0) $fatal(1,"Window delayed safety disable");
        $display("V2_FIVE_SAMPLE_WINDOW_PASS"); $finish;
    end
endmodule
