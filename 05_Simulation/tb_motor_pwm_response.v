`timescale 1ns/1ps
module tb_motor_pwm_response;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,en=0,fault=0;
    reg signed [15:0] command=0;
    wire in1,in2,pwm;
    localparam P=50,BLANK=50;
    motor_signed_pwm #(.PWM_PERIOD(P),.REVERSE_BLANK_CYCLES(BLANK)) dut (
        .clk(clk),.rst_n(rst_n),.drive_enable(en),.fault(fault),.command(command),
        .in1(in1),.in2(in2),.pwm(pwm));
    integer n,k,phase,polarity,high_count,first_on,width;
    task cycle;begin @(posedge clk);#1;end endtask
    task cycles;input integer count;integer j;begin for(j=0;j<count;j=j+1)cycle;end endtask
    task require_off;begin #1;if(in1||in2||pwm)$fatal(1,"Inhibit is not immediate");end endtask
    task reset;
        begin @(negedge clk);rst_n=0;en=0;fault=0;command=0;cycles(3);
            @(negedge clk);rst_n=1;cycles(3);end
    endtask
    initial begin #100000000;$fatal(1,"PWM response timeout");end
    initial begin
        // Exhaust all carrier phases in both directions. Every resumed first
        // pulse must have full commanded width after the existing blank.
        for(polarity=-1;polarity<=1;polarity=polarity+2)begin
            for(phase=0;phase<P;phase=phase+1)begin
                reset;@(negedge clk);en=1;command=12*polarity;cycle;
                if(!pwm||in1!==(polarity>0)||in2!==(polarity<0))$fatal(1,"Start phase delay");
                high_count=0;
                for(k=0;k<P;k=k+1)begin if(pwm)high_count=high_count+1;cycle;end
                if(high_count!==12)$fatal(1,"Duty changed");
                cycles(phase);
                @(negedge clk);command=-12*polarity;require_off;
                first_on=0;width=0;
                for(k=1;k<=BLANK+P+2;k=k+1)begin
                    cycle;
                    if(k<=BLANK&&(in1||in2||pwm))$fatal(1,"Blank too short");
                    if(pwm&&first_on==0)first_on=k;
                    if(k>=BLANK+1&&k<BLANK+1+12&&pwm)width=width+1;
                end
                if(first_on!==BLANK+1||width!==12)$fatal(1,"Reverse phase delay or partial pulse");
            end
        end
        // A short zero or enable gap cannot bypass reversal blanking.
        for(n=0;n<2;n=n+1)begin
            reset;@(negedge clk);en=1;command=20;cycles(70);
            @(negedge clk);if(n==0)command=0;else en=0;require_off;cycles(7);
            @(negedge clk);en=1;command=-20;require_off;
            for(k=0;k<BLANK-7;k=k+1)begin cycle;if(in1||in2||pwm)$fatal(1,"Short gap bypassed blank");end
            cycles(3);if(in1||!in2||!pwm)$fatal(1,"Short-gap reverse resume");
        end
        // Reversing again while blanked must wait out that same interval.
        reset;@(negedge clk);en=1;command=20;cycles(70);
        @(negedge clk);command=-20;require_off;cycles(10);
        @(negedge clk);command=20;require_off;cycles(10);
        @(negedge clk);command=-20;require_off;cycles(10);
        fault=1;require_off;cycles(80);fault=0;cycle;
        if(in1||!in2||!pwm)$fatal(1,"Fault resume direction or phase");
        en=0;require_off;cycles(80);command=20;en=1;cycle;
        if(!in1||in2||!pwm)$fatal(1,"Long-idle direction switch");
        @(negedge clk);command=32767;cycles(60);
        repeat(P)begin cycle;if(!pwm)$fatal(1,"Positive full-duty clamp");end
        @(negedge clk);command=-32768;cycles(60);
        repeat(P)begin cycle;if(!pwm||in1||!in2)$fatal(1,"Negative full-duty clamp");end
        fault=1;require_off;cycles(3);rst_n=0;require_off;
        $display("V2_MOTOR_PWM_RESPONSE_PASS");$finish;
    end
endmodule
