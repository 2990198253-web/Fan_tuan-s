`timescale 1ns/1ps
module tb_half_adc;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, alternating=1, phase=0;
    reg [9:0] data=763, constant_data=763;
    wire [9:0] whole;
    wire [10:0] half_value;
    wire valid;
    adc_capture #(.SAMPLE_DIV(20),.AVERAGE_LOG2(4)) adc (
        .clk(clk),.rst_n(rst_n),.adc_data(data),.angle_raw(whole),.angle_x2(half_value),.angle_valid(valid));
    always @(negedge clk) begin
        if(!rst_n) begin phase=0; data=763; end
        else if(adc.sample_divider==0 && !adc.adc_clk) begin
            if(alternating) begin data=phase?764:763; phase=!phase; end
            else data=constant_data;
        end
    end
    task cycles; input integer count; integer n;
        begin for(n=0;n<count;n=n+1) begin @(posedge clk); #1; end end endtask
    initial begin #100000; $fatal(1,"Half ADC timeout"); end
    initial begin
        cycles(3); rst_n=1; cycles(360);
        if(!valid || whole!==764 || half_value!==1527) $fatal(1,"763/764 average lost its half count");
        alternating=0; constant_data=763; cycles(360);
        if(whole!==763 || half_value!==1526) $fatal(1,"763 scale");
        constant_data=764; cycles(360); if(half_value!==1528) $fatal(1,"764 scale");
        constant_data=1023; cycles(360); if(whole!==1023 || half_value!==2046) $fatal(1,"Half ADC full-scale overflow");
        constant_data=0; cycles(360); if(whole!==0 || half_value!==0) $fatal(1,"Half ADC low-scale underflow");
        $display("V2_HALF_ADC_PASS midpoint=763.5 x2=1527"); $finish;
    end
endmodule
