`timescale 1ns/1ps
module tb_measured_12v;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,update=0,start=0,stop=0,estop=0;
    reg [10:0] angle=421;
    reg signed [31:0] position=0;
    integer angles[0:23],positions[0:23];
    wire signed [15:0] command;wire fault;wire [3:0] reason;
    swing_up_supervisor dut(.clk(clk),.rst_n(rst),.update(update),.start(start),.stop(stop),.estop(estop),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(angle),.position(position),
        .swing_command(command),.fault(fault),.reason(reason));
    task tick;begin @(negedge clk);update=1;@(negedge clk);update=0;@(negedge clk);end endtask
    integer i,j,mirrored,negative_brakes,positive_brakes;
    initial begin
        `include "measured_12v_swing.vh"
        negative_brakes=0;positive_brakes=0;
        $dumpfile("smooth_results/v6r2_measured_12v.vcd");
        $dumpvars(0,clk,position,angle,command,dut.arm_velocity_counts,dut.braking,dut.returning);
        for(mirrored=0;mirrored<2;mirrored=mirrored+1)begin
            @(negedge clk);rst=0;position=0;repeat(3)@(negedge clk);rst=1;
            start=1;@(negedge clk);start=0;
            for(i=0;i<24;i=i+1)begin
                angle=angles[i];position=mirrored ? -positions[i] : positions[i];
                // Telemetry is 100 Hz, control is 2 kHz: 20 control ticks/frame.
                for(j=0;j<20;j=j+1)tick();
                if(command>1400||command < -1400)$fatal(1,"V6 R2 swing output cap");
                if(positions[i]>=150)begin
                    if(!mirrored && command>=0)$fatal(1,"Measured positive travel still driven outward");
                    if(mirrored && command<=0)$fatal(1,"Mirrored travel still driven outward");
                end
                if(!mirrored && dut.overspeed && command<0)negative_brakes=negative_brakes+1;
                if(mirrored && dut.overspeed && command>0)positive_brakes=positive_brakes+1;
                if(fault)$fatal(1,"Fixture before 240 counts unexpectedly faulted");
            end
            if(!mirrored && negative_brakes==0)$fatal(1,"No measured overspeed brake");
            if(mirrored && positive_brakes==0)$fatal(1,"No mirrored overspeed brake");
            stop=1;#1;if(command!==0)$fatal(1,"Stop must override active brake");
            @(negedge clk);stop=0;repeat(3)@(negedge clk);
        end
        // Isolate predictive return from the speed guard. At +100 counts and
        // +10 counts/sample, +80ms prediction crosses the unchanged +150 limit.
        @(negedge clk);rst=0;position=0;angle=421;repeat(3)@(negedge clk);rst=1;
        start=1;@(negedge clk);start=0;
        force dut.overspeed=1'b0;force dut.braking=1'b0;force dut.arm_velocity_counts=33'sd10;
        position=100;tick();if(command>=0)$fatal(1,"Predictive brake was not applied before +150");
        release dut.overspeed;release dut.braking;release dut.arm_velocity_counts;
        $display("V6R2_MEASURED_12V_PASS measured_24_frames mirrored_24_frames inward_return predictive_return overspeed_brake stop_priority");
        $finish;
    end
    initial begin #1000000;$fatal(1,"Measured replay timeout");end
endmodule
