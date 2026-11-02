`timescale 1ns/1ps
// Recorded input replay tests decisions, not the response of a physical plant.
module tb_capture_replay;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,up=0,start=0;reg [10:0] angle=433;reg signed [31:0] pos=0;
    wire signed [15:0] cmd;wire [2:0] state;wire [3:0] reason;
    reg [63:0] data[0:362];integer i,j,file_out,pre_frames=0,top_frames=0,rail_frames=0;
    reg [223:0] cfg={16'd20,16'd1000,16'd15000,16'd2,16'd97,16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
    swing_up_supervisor #(.LAB_ENABLE(1)) dut(
        .clk(clk),.rst_n(rst),.update(up),.start(start),.stop(1'b0),.estop(1'b0),
        .sensor_fresh(1'b1),.encoder_illegal(1'b0),.angle_x2(angle),.position(pos),.lab_config_words(cfg),
        .swing_command(cmd),.state(state),.reason(reason));
    task tick;begin @(negedge clk);up=1;@(negedge clk);up=0;@(negedge clk);end endtask
    initial begin
        $readmemh("measured_lab3_213400.hex",data);
        file_out=$fopen("smooth_results/r1_recorded_decisions.csv","w");
        $fdisplay(file_out,"frame,angle_x2,recorded_encoder,recorded_motor,new_motor,new_state,angle_valid");
        $dumpfile("smooth_results/r1_recorded_replay.vcd");$dumpvars(0,cmd,state,angle,pos,dut.catch_observation_valid);
        repeat(3)@(negedge clk);rst=1;start=1;@(negedge clk);start=0;
        for(i=0;i<363;i=i+1)begin
            angle=data[i][63:48];pos=$signed(data[i][47:16]);
            for(j=0;j<20;j=j+1)begin tick();
                if((angle<=8||angle>=2038) && cmd!=0)$fatal(1,"Recorded rail drives motor at frame %0d",i);
            end
            if(state==5)pre_frames=pre_frames+1;
            if(angle>=1470&&angle<=1550&&i<300)begin
                top_frames=top_frames+1;
                if(state!=5)$fatal(1,"Recorded first top not under precapture %0d state=%0d",i,state);
                if(cmd<=0)$fatal(1,"Positive pole speed not damped with balance polarity");
            end
            if(angle<=8||angle>=2038)rail_frames=rail_frames+1;
            $fdisplay(file_out,"%0d,%0d,%0d,%0d,%0d,%0d,%0d",i,angle,pos,$signed(data[i][15:0]),cmd,state,dut.catch_observation_valid);
            if(state==2)$fatal(1,"Replay falsely claims original fast trajectory caught");
        end
        if(pre_frames<5||top_frames<2||rail_frames<10||state!=3||reason!=11)$fatal(1,"Replay coverage pre=%0d top=%0d rail=%0d state=%0d reason=%0d",pre_frames,top_frames,rail_frames,state,reason);
        $fclose(file_out);
        $display("LAB3_R1_REPLAY_PASS recorded363_frames pre=%0d top=%0d rail=%0d no_false_balance valid_gated retained_ADC_fault",pre_frames,top_frames,rail_frames);$finish;
    end
    initial begin #5000000;$fatal(1,"Replay timeout");end
endmodule
