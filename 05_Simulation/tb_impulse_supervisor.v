`timescale 1ns/1ps
module tb_impulse_supervisor;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,update=0,start=0,stop=0,estop=0,fresh=1,illegal=0;
    reg [10:0] angle=450;reg signed [31:0] pos=0;
    wire signed [15:0] command;wire signed [7:0] target;
    wire [3:0] reason,action;wire fault,balancing;
    swing_up_supervisor #(.IMPULSE_ENABLE(1),.TRAJECTORY_ENABLE(1),
        .VELOCITY_TICKS(2),.TIMEOUT_TICKS(100)) dut(
        .clk(clk),.rst_n(rst),.update(update),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(fresh),.encoder_illegal(illegal),.angle_x2(angle),.position(pos),
        .swing_command(command),.fault(fault),.reason(reason),.balancing(balancing),
        .swing_action(action),.swing_target_offset(target));
    task tick;begin @(negedge clk);update=1;@(negedge clk);update=0;@(negedge clk);end endtask
    task ticks;input integer n;integer k;begin for(k=0;k<n;k=k+1)tick();end endtask
    task restart;begin rst=0;stop=0;estop=0;fresh=1;illegal=0;pos=0;angle=450;
        repeat(3)@(negedge clk);rst=1;start=1;@(negedge clk);start=0;ticks(4);end endtask
    initial begin
        restart();if(command!=4000||target!=0)$fatal(1,"R5 not selected, or slow ramp active");
        force dut.drive_hold=-16'sd1400;force dut.trajectory_command=-16'sd1400;ticks(2);
        if(command!=4000)$fatal(1,"Old controller overrode R5");
        release dut.drive_hold;release dut.trajectory_command;
        stop=1;#1;if(command!==0)$fatal(1,"Raw stop gate");
        restart();estop=1;#1;if(command!==0)$fatal(1,"Raw emergency gate");
        @(negedge clk);if(reason!=2)$fatal(1,"Emergency latch");
        restart();fresh=0;#1;if(command!==0)$fatal(1,"ADC freshness gate");
        @(negedge clk);if(!fault||reason!=4)$fatal(1,"ADC fault");
        restart();illegal=1;#1;if(command!==0)$fatal(1,"Encoder gate");
        @(negedge clk);if(!fault||reason!=7)$fatal(1,"Encoder latch");
        restart();pos=240;#1;if(command!==0)$fatal(1,"Positive hard travel gate");
        @(negedge clk);if(!fault||reason!=6)$fatal(1,"Travel latch");
        restart();pos=-240;#1;if(command!==0)$fatal(1,"Negative hard travel gate");
        restart();ticks(100);if(!fault||reason!=5||command!=0)$fatal(1,"Timeout");
        restart();angle=0;ticks(4);if(command!=0||action!=9)$fatal(1,"ADC rail caused kick");
        restart();angle=1510;ticks(16);if(!balancing||fault||command!=0)$fatal(1,"Balance handoff");
        $display("V6R5_SUPERVISOR_PASS selected safety stop estop sensor encoder travel timeout rail handoff");$finish;
    end
    initial begin #500000;$fatal(1,"Supervisor timeout");end
endmodule
