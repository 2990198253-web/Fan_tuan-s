`timescale 1ns/1ps
// Arm-only smoke model, NOT an identified Furuta/pendulum model.
// Measured sign: +command -> +encoder. dt=0.5ms. The gain/drag are assumptions.
module tb_arm_inertia;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,update=0,start=0,stop=0;
    reg signed [31:0] pos_new=0,pos_old=0;
    wire signed [15:0] u_new,u_old;wire fault_new,fault_old;
    wire [3:0] reason_new,reason_old;
    swing_up_supervisor fixed_dut(.clk(clk),.rst_n(rst),.update(update),.start(start),.stop(stop),.estop(1'b0),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(11'd450),.position(pos_new),
        .swing_command(u_new),.fault(fault_new),.reason(reason_new));
    swing_up_supervisor_v6_reference old_dut(.clk(clk),.rst_n(rst),.update(update),.start(start),.stop(stop),.estop(1'b0),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(11'd450),.position(pos_old),
        .swing_command(u_old),.fault(fault_old),.reason(reason_old));
    real x_new=0.0,x_old=0.0,v_new=0.0,v_old=0.0,max_new=0.0,max_old=0.0,max_speed=0.0;
    real fault_time=-1.0;
    integer i,reverse_ticks=0;
    function real absolute;input real value;begin absolute=value<0 ? -value : value;end endfunction
    initial begin
        $dumpfile("smooth_results/v61_arm_inertia.vcd");
        $dumpvars(0,pos_new,pos_old,u_new,u_old,fault_new,fault_old);
        repeat(3)@(negedge clk);rst=1;start=1;@(negedge clk);start=0;
        for(i=0;i<10000;i=i+1)begin
            // Electrical torque surrogate with positive encoder sign and inertia.
            v_new=v_new+(3.0*$itor(u_new)-1.5*v_new)*0.0005;x_new=x_new+v_new*0.0005;
            v_old=v_old+(3.0*$itor(u_old)-1.5*v_old)*0.0005;x_old=x_old+v_old*0.0005;
            pos_new=$rtoi(x_new);pos_old=$rtoi(x_old);
            if(absolute(x_new)>max_new)max_new=absolute(x_new);
            if(absolute(x_old)>max_old)max_old=absolute(x_old);
            if(absolute(v_new)>max_speed)max_speed=absolute(v_new);
            if(u_new<0)reverse_ticks=reverse_ticks+1;
            @(negedge clk);update=1;@(negedge clk);update=0;@(negedge clk);
            if(fault_new)$fatal(1,"Corrected measured-sign arm model hit protection");
            if(fault_old&&fault_time<0)fault_time=i*0.0005;
        end
        if(!fault_old||reason_old!=6||max_old<240.0)$fatal(1,"Old bad return sign was not reproduced");
        if(max_new>=240.0||max_speed>=800.0||reverse_ticks==0)$fatal(1,"New travel/speed/brake model invariant");
        stop=1;#1;if(u_new!==0)$fatal(1,"Stop active-brake override");
        $display("V61_ARM_INERTIA_PASS old_fault_s=%f old_peak_counts=%f fixed_peak_counts=%f fixed_peak_counts_per_s=%f",fault_time,max_old,max_new,max_speed);
        $finish;
    end
    initial begin #5000000;$fatal(1,"Inertia smoke timeout");end
endmodule
