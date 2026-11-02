// 64-byte snapshot at 100 Hz. CRC covers bytes2..61, little-endian CRC tail.
module lab_telemetry #(parameter integer CLOCK_HZ=50000000,BAUD=115200,FRAME_DIV=500000)(
    input wire clk,rst_n,enable,input wire [495:0] packet,
    output wire tx
);
    reg [1:0] state;reg [31:0] count;reg [5:0] index;
    reg [7:0] seq_no;reg [495:0] hold;reg [495:0] packet_snapshot;reg [15:0] crc;
    reg start;reg [7:0] data;wire busy,done;
    wire [7:0] selected=index<62 ? hold[index*8+:8] : index==62 ? crc[7:0] : crc[15:8];
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] val;reg [15:0] w;integer j;
        begin w=old^{val,8'd0};for(j=0;j<8;j=j+1)w=w[15] ? (w<<1)^16'h1021 : w<<1;crc_byte=w;end
    endfunction
    uart_tx #(.CLOCK_HZ(CLOCK_HZ),.BAUD(BAUD)) uart(.clk(clk),.rst_n(rst_n),.start(start),.data(data),.tx(tx),.busy(busy),.done(done));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=0;count<=0;index<=0;seq_no<=0;hold<=0;packet_snapshot<=0;crc<=16'hFFFF;start<=0;data<=0;end
        else begin
            // Isolate the live configuration/diagnostic bus from the UART
            // frame latch. The original direct 496-bit path missed 50 MHz.
            packet_snapshot<=packet;
            start<=0;if(count==FRAME_DIV-1)count<=0;else count<=count+1'b1;
            case(state)
                0:if(enable && count==FRAME_DIV-1)begin
                    hold<=packet_snapshot;hold[31:24]<=seq_no;index<=0;crc<=16'hFFFF;state<=1;end
                1:if(!busy)begin
                    data<=selected;start<=1;if(index>=2 && index<=61)crc<=crc_byte(crc,selected);state<=2;end
                2:if(done)begin
                    if(index==63)begin seq_no<=seq_no+1'b1;state<=0;end
                    else begin index<=index+1'b1;state<=1;end
                end
            endcase
        end
    end
endmodule
