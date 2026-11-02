`timescale 1ns/1ps
module tb_energy_supervisor;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,up=0,start=0,stop=0,estop=0,fresh=1,illegal=0;
    reg [10:0] angle=450;reg signed [31:0] pos=0;reg [223:0] cfg;
    wire signed [15:0] cmd,estimate;wire [3:0] reason,action;wire balance,fault;
    swing_up_supervisor #(.LAB_ENABLE(1),.VELOCITY_TICKS(2)) dut(
        .clk(clk),.rst_n(rst),.update(up),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(fresh),.encoder_illegal(illegal),.angle_x2(angle),.position(pos),.lab_config_words(cfg),
        .swing_command(cmd),.reason(reason),.swing_action(action),.balancing(balance),.fault(fault),.lab_energy_proxy(estimate));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    task ticks;input integer n;integer i;begin for(i=0;i<n;i=i+1)tick();end endtask
    task restart;begin rst=0;up=0;stop=0;estop=0;fresh=1;illegal=0;angle=450;pos=0;
        cfg={16'd20,16'd1000,16'd15000,16'd2,16'd17,16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
        repeat(3)@(negedge clk);rst=1;start=1;@(negedge clk);start=0;ticks(4);end endtask
    initial begin
        restart();if(cmd!=2500||action!=15||!dut.energy_mode)$fatal(1,"New mode not selected");
        force dut.pulse_command=-16'sd100;tick();if(cmd<=0)$fatal(1,"Old pulse overrides energy mode");release dut.pulse_command;
        stop=1;#1;if(cmd!=0)$fatal(1,"Raw stop gate");
        restart();estop=1;#1;if(cmd!=0)$fatal(1,"Raw emergency gate");
        restart();illegal=1;#1;if(cmd!=0)$fatal(1,"Encoder gate");@(negedge clk);if(!fault||reason!=7)$fatal(1,"Encoder latch");
        restart();fresh=0;#1;if(cmd!=0)$fatal(1,"ADC gate");
        restart();pos=240;#1;if(cmd!=0)$fatal(1,"Positive hard travel");
        restart();pos=-240;#1;if(cmd!=0)$fatal(1,"Negative hard travel");
        restart();angle=1510;ticks(16);if(!balance||fault||cmd!=0)$fatal(1,"Existing upright handoff lost");
        restart();cfg[9*16+:16]=16;angle=1510;ticks(16);if(balance)$fatal(1,"Disabled auto-balance ignored");
        $display("SWING_LAB2_SUPERVISOR_PASS selected stop estop encoder ADC both_limits auto_balance optional_balance");$finish;
    end
    initial begin #1000000;$fatal(1,"Energy supervisor timeout");end
endmodule
