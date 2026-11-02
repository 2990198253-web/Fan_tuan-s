`timescale 1ns/1ps
module tb_lab_config;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,valid=0,fe=0,safe=1,safe_target=0,safe_tune=0;reg [7:0] data=0;
    wire [223:0] cfg;wire [7:0] ack,seq;wire [15:0] rev,errors;wire stop,seen,target_request,gain_request,inner_gain_request;
    wire signed [15:0] target_relative;wire [15:0] target_speed;
    wire [15:0] gain_p_q8,gain_i_q16,gain_d_q8;wire gain_i_present;
    wire [15:0] inner_p,inner_i_q8,inner_d;
    lab_config #(.CLOCK_HZ(2000)) dut(.clk(clk),.rst_n(rst),.byte_valid(valid),.frame_error(fe),
        .safe_to_config(safe),.safe_to_target(safe_target),.safe_to_tune(safe_tune),
        .byte_data(data),.config_words(cfg),.ack_sequence(seq),.ack_status(ack),
        .revision(rev),.crc_errors(errors),.remote_stop(stop),.packet_seen(seen),
        .target_request(target_request),.target_relative(target_relative),.target_speed(target_speed),
        .gain_request(gain_request),.gain_i_present(gain_i_present),
        .gain_p_q8(gain_p_q8),.gain_i_q16(gain_i_q16),.gain_d_q8(gain_d_q8),
        .inner_gain_request(inner_gain_request),.inner_p(inner_p),.inner_i_q8(inner_i_q8),.inner_d(inner_d));
    reg [7:0] frame[0:39];reg [223:0] candidate;integer i,k,stops=0,targets=0,gains=0,inner_gains=0;
    always @(posedge clk)if(stop)stops=stops+1;
    always @(posedge clk)if(target_request)targets=targets+1;
    always @(posedge clk)if(gain_request)gains=gains+1;
    always @(posedge clk)if(inner_gain_request)inner_gains=inner_gains+1;
    function [15:0] crc_byte;input [15:0] old;input [7:0] v;reg [15:0] w;integer j;
        begin w=old^{v,8'd0};for(j=0;j<8;j=j+1)w=w[15] ? (w<<1)^16'h1021 : w<<1;crc_byte=w;end endfunction
    task make;input [7:0] seq_no,opcode;reg [15:0] crc;begin
        for(i=0;i<40;i=i+1)frame[i]=0;frame[0]=8'hC3;frame[1]=8'h3C;frame[2]=8'h61;frame[3]=seq_no;frame[4]=opcode;
        for(i=0;i<28;i=i+1)frame[6+i]=candidate[i*8+:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task move_frame;input [7:0] seq_no;input signed [15:0] rel;input [15:0] speed;reg [15:0] crc;begin
        make(seq_no,4);frame[34]=rel[7:0];frame[35]=rel[15:8];
        frame[36]=speed[7:0];frame[37]=speed[15:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task gain_frame;input [7:0] seq_no;input [15:0] p,d;reg [15:0] crc;begin
        make(seq_no,5);frame[34]=p[7:0];frame[35]=p[15:8];
        frame[36]=d[7:0];frame[37]=d[15:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task pid_frame;input [7:0] seq_no;input [15:0] p,ki,d;reg [15:0] crc;begin
        make(seq_no,6);frame[32]=ki[7:0];frame[33]=ki[15:8];
        frame[34]=p[7:0];frame[35]=p[15:8];
        frame[36]=d[7:0];frame[37]=d[15:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task inner_frame;input [7:0] seq_no;input [15:0] p,ki,d;reg [15:0] crc;begin
        make(seq_no,7);frame[32]=ki[7:0];frame[33]=ki[15:8];
        frame[34]=p[7:0];frame[35]=p[15:8];
        frame[36]=d[7:0];frame[37]=d[15:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task byte_send;input [7:0] b;begin @(negedge clk);data=b;valid=1;@(negedge clk);valid=0;end endtask
    task send;begin for(i=0;i<40;i=i+1)byte_send(frame[i]);repeat(3)@(negedge clk);end endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;@(negedge clk);candidate=cfg;
        if(cfg[6*16+:16]!=8||cfg[9*16+:16]!=8)$fatal(1,"Bad defaults");
        candidate[0+:16]=240;candidate[6*16+:16]=12;make(7,1);
        for(i=0;i<39;i=i+1)byte_send(frame[i]);if(rev!=0||cfg[0+:16]!=120)$fatal(1,"Non-atomic commit");
        byte_send(frame[39]);repeat(3)@(negedge clk);
        if(rev!=1||cfg!=candidate||ack!=1||seq!=7)$fatal(1,"Valid config not committed/read back");
        safe=0;candidate[0+:16]=300;make(8,1);send();if(ack!=2||rev!=1||cfg[0+:16]!=240)$fatal(1,"Busy accepted configuration");
        safe=1;candidate[0+:16]=241;make(9,1);send();if(ack!=3||rev!=1)$fatal(1,"Non-10ms accepted");
        candidate=cfg;candidate[6*16+:16]=21;make(10,1);send();if(ack!=3||rev!=1)$fatal(1,"Speed range accepted");
        candidate=cfg;make(11,1);frame[20]=frame[20]^1;send();if(errors!=1||seq!=10||rev!=1)$fatal(1,"CRC corrupted command accepted");
        make(12,3);send();if(stops!=1||seq!=12||ack!=1)$fatal(1,"Remote STOP");
        make(13,2);send();if(seq!=13||ack!=1||rev!=1)$fatal(1,"Query changed parameters");
        move_frame(70,16'sd50,16'd10);send();
        if(ack!=2||targets!=0||rev!=1)$fatal(1,"Target accepted outside BALANCE");
        safe_target=1;move_frame(71,-16'sd50,16'd12);send();
        if(ack!=1||targets!=1||target_relative!==-16'sd50||target_speed!==12||rev!=1)
            $fatal(1,"Valid target not accepted atomically");
        move_frame(72,16'sd181,16'd12);send();if(ack!=3||targets!=1)$fatal(1,"Travel limit bypassed");
        move_frame(73,16'sd20,16'd31);send();if(ack!=3||targets!=1)$fatal(1,"Speed limit bypassed");
        gain_frame(74,16'd12,16'd512);send();if(ack!=2||gains!=0)$fatal(1,"Unsafe tuning accepted");
        safe_tune=1;gain_frame(75,16'd12,16'd512);send();
        if(ack!=1||gains!=1||gain_p_q8!=12||gain_d_q8!=512||rev!=1)
            $fatal(1,"Runtime gains not accepted atomically");
        gain_frame(76,16'd33,16'd512);send();if(ack!=3||gains!=1)$fatal(1,"P range bypassed");
        gain_frame(77,16'd12,16'd8192);send();if(ack!=3||gains!=1)$fatal(1,"D range bypassed");
        safe_tune=0;pid_frame(78,16'd12,16'd4,16'd512);send();if(ack!=2||gains!=1)$fatal(1,"Unsafe PID tuning accepted");
        safe_tune=1;pid_frame(79,16'd16,16'd4,16'd512);send();
        if(ack!=1||gains!=2||gain_p_q8!=16||gain_i_q16!=4||gain_d_q8!=512||rev!=1)
            $fatal(1,"Runtime PID gains not accepted atomically");
        pid_frame(80,16'd16,16'd65,16'd512);send();if(ack!=3||gains!=2)$fatal(1,"I range bypassed");
        pid_frame(81,16'd33,16'd4,16'd512);send();if(ack!=3||gains!=2)$fatal(1,"PID P range bypassed");
        pid_frame(82,16'd16,16'd4,16'd8192);send();if(ack!=3||gains!=2)$fatal(1,"PID D range bypassed");
        safe_tune=0;inner_frame(83,16'd220,16'd16,16'd400);send();
        if(ack!=2||inner_gains!=0)$fatal(1,"Unsafe inner PID tuning accepted");
        safe_tune=1;inner_frame(84,16'd220,16'd16,16'd400);send();
        if(ack!=1||inner_gains!=1||inner_p!=220||inner_i_q8!=16||inner_d!=400||rev!=1)
            $fatal(1,"Inner PID not accepted atomically");
        inner_frame(85,16'd1001,16'd16,16'd400);send();if(ack!=3||inner_gains!=1)$fatal(1,"Inner P range bypassed");
        inner_frame(86,16'd220,16'd129,16'd400);send();if(ack!=3||inner_gains!=1)$fatal(1,"Inner I range bypassed");
        inner_frame(87,16'd220,16'd16,16'd1201);send();if(ack!=3||inner_gains!=1)$fatal(1,"Inner D range bypassed");
        make(14,8);send();if(ack!=4||stops!=1)$fatal(1,"Unknown opcode accepted");
        byte_send(8'hC3);byte_send(8'h3C);repeat(110)@(negedge clk);
        make(15,2);send();if(seq!=15||ack!=1)$fatal(1,"Partial frame timeout recovery");
        byte_send(8'hC3);fe=1;@(negedge clk);fe=0;make(16,2);send();if(seq!=16)$fatal(1,"Framing recovery");
        for(k=0;k<14;k=k+1)begin
            candidate=cfg;candidate[k*16+:16]=65535;make(20+k,1);send();
            if(ack!=3||rev!=1)$fatal(1,"A parameter field bypassed its range check");
        end
        candidate=cfg;candidate[9*16+:16]=49;make(40,1);send();
        if(ack!=3||rev!=1)$fatal(1,"Energy/relay conflict accepted");
        candidate[9*16+:16]=33;make(41,1);send();
        if(ack!=1||rev!=2||cfg[9*16+:16]!=33)$fatal(1,"Relay flags rejected");
        candidate[9*16+:16]=65;make(42,1);send();if(ack!=3||rev!=2)$fatal(1,"Catch without relay accepted");
        candidate[9*16+:16]=97;make(43,1);send();if(ack!=1||rev!=3||cfg[9*16+:16]!=97)$fatal(1,"R1 flag not committed");
        candidate[9*16+:16]=129;candidate[6*16+:16]=24;candidate[7*16+:16]=350;make(44,1);send();
        if(ack!=1||rev!=4||cfg!=candidate)$fatal(1,"Reference profile rejected");
        candidate[9*16+:16]=133;make(45,1);send();if(ack!=3||rev!=4)$fatal(1,"Reference reverse conflicts accepted");
        candidate[9*16+:16]=33;make(46,1);send();if(ack!=3||rev!=4)$fatal(1,"Legacy extended range accepted");
        safe_tune=1;pid_frame(99,16'd16,16'd4,16'd8191);send();
        if(ack!=1||gain_d_q8!=8191||gains!=3)$fatal(1,"Maximum outer D was not accepted");
        $display("SWING_LAB_CONFIG_PASS atomic ACK busy CRC range timeout framing query stop no_start relay_flags mutually_exclusive precapture_flag reference_ranges");$finish;
    end
    initial begin #1000000;$fatal(1,"Lab config timeout");end
endmodule
