`timescale 1ns/1ps
module tb_locked_config;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,valid=0,fe=0,safe=1;reg [7:0] data=0;
    wire [223:0] cfg;wire [7:0] ack,seq;wire [15:0] rev,errors;wire stop,seen;
    lab_config #(.CLOCK_HZ(2000),.REFERENCE_DEFAULT(1),.REFERENCE_ONLY(1)) dut(.clk(clk),.rst_n(rst),.byte_valid(valid),.frame_error(fe),
        .safe_to_config(safe),.byte_data(data),.config_words(cfg),.ack_sequence(seq),.ack_status(ack),
        .revision(rev),.crc_errors(errors),.remote_stop(stop),.packet_seen(seen));
    reg [7:0] frame[0:39];reg [223:0] candidate;integer i,k,stops=0;
    always @(posedge clk)if(stop)stops=stops+1;
    function [15:0] crc_byte;input [15:0] old;input [7:0] v;reg [15:0] w;integer j;
        begin w=old^{v,8'd0};for(j=0;j<8;j=j+1)w=w[15] ? (w<<1)^16'h1021 : w<<1;crc_byte=w;end endfunction
    task make;input [7:0] seq_no,opcode;reg [15:0] crc;begin
        for(i=0;i<40;i=i+1)frame[i]=0;frame[0]=8'hC3;frame[1]=8'h3C;frame[2]=8'h61;frame[3]=seq_no;frame[4]=opcode;
        for(i=0;i<28;i=i+1)frame[6+i]=candidate[i*8+:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
    end endtask
    task byte_send;input [7:0] b;begin @(negedge clk);data=b;valid=1;@(negedge clk);valid=0;end endtask
    task send;begin for(i=0;i<40;i=i+1)byte_send(frame[i]);repeat(3)@(negedge clk);end endtask

    initial begin
        repeat(3)@(negedge clk);rst=1;@(negedge clk);candidate=cfg;
        if(cfg[9*16+:16]!=129||cfg[7*16+:16]!=350)$fatal(1,"Locked reference defaults");
        candidate[0+:16]=80;make(1,1);send();if(ack!=1||rev!=1)$fatal(1,"Reference parameters rejected");
        for(k=0;k<6;k=k+1)begin
            candidate=cfg;
            case(k)
                0:candidate[9*16+:16]=8;
                1:candidate[9*16+:16]=17;
                2:candidate[9*16+:16]=33;
                3:candidate[9*16+:16]=97;
                4:candidate[9*16+:16]=131;
                5:candidate[9*16+:16]=133;
            endcase
            make(10+k,1);send();
            if(ack!=3||rev!=1||cfg[9*16+:16]!=129)$fatal(1,"Legacy strategy escaped lock");
        end
        candidate=cfg;candidate[9*16+:16]=128;make(20,1);send();
        if(ack!=1||rev!=2||cfg[9*16+:16]!=128)$fatal(1,"Optional balance rejected");
        candidate[9*16+:16]=129;make(21,1);send();if(ack!=1||rev!=3)$fatal(1,"Reference balance enable rejected");
        make(22,3);send();if(stops!=1)$fatal(1,"Locked STOP failed");
        $display("LAB4_LOCKED_CONFIG_PASS defaults129 reference_tuning old_modes_rejected atomic optional_balance stop");$finish;
    end
    initial begin #1000000;$fatal(1,"Locked config timeout");end
endmodule
