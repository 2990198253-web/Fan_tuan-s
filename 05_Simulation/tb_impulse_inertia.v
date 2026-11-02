`timescale 1ns/1ps
// Hypothetical arm-only gains. No pole model and no claim of swing-up success.
module tb_impulse_inertia;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,en=0,up=0,phase=1;
    reg signed [32:0] pos=0,vel=0;
    wire signed [15:0] cmd;wire [3:0] action;
    impulse_swing_controller dut(.clk(clk),.rst_n(rst),.enable(en),.update(up),
        .displacement(pos),.arm_velocity(vel),.phase_positive(phase),.phase_valid(1'b1),
        .angle_valid(1'b1),.capture_zone(1'b0),.command(cmd),.action(action));
    integer i,c,last,brakes,kicks,guards;
    real x,v,gain,peak,speed;
    function real abs;input real z;begin abs=z<0 ? -z : z;end endfunction
    initial begin
        $dumpfile("smooth_results/r5_inertia.vcd");$dumpvars(0,pos,vel,cmd,action);
        for(c=0;c<3;c=c+1)begin
            gain=c==0 ? 3.0 : c==1 ? 6.0 : 12.0;
            rst=0;en=0;up=0;pos=0;vel=0;x=0;v=0;peak=0;speed=0;last=0;brakes=0;kicks=0;guards=0;
            repeat(3)@(negedge clk);rst=1;en=1;
            for(i=0;i<20000;i=i+1)begin
                v=v+(gain*$itor(cmd)-1.5*v)*0.0005;x=x+v*0.0005;pos=$rtoi(x);
                if(abs(x)>peak)peak=abs(x);if(abs(v)>speed)speed=abs(v);
                if(abs(x)>=240)$fatal(1,"Assumed arm exceeded unchanged hard travel");
                if(i%20==0)begin
                    vel=pos-last;last=pos;phase=(i/800)%2==0;
                    @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);
                    if(action==4)brakes=brakes+1;if(action==2||action==3)kicks=kicks+1;
                    if(action==7)guards=guards+1;
                    if(cmd>4000||cmd < -4000)$fatal(1,"Impulse effort bound");
                end
            end
            if(kicks<5||brakes<5)$fatal(1,"Missing repeated kick/brake actions");
            $display("R5_ARM_ONLY gain=%f peak_counts=%f speed_counts_per_s=%f kick_samples=%0d brake_samples=%0d guard_samples=%0d",gain,peak,speed,kicks,brakes,guards);
            en=0;@(negedge clk);if(cmd!=0)$fatal(1,"Disable during inertia model");
        end
        $display("V6R5_INERTIA_PASS assumed_gains_only effort travel kick brake disable");$finish;
    end
    initial begin #5000000;$fatal(1,"Inertia test timeout");end
endmodule
