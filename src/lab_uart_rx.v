// 8N1 centre sampling, synchronized input, false-start/stop-bit rejection.
module lab_uart_rx #(parameter integer CLOCK_HZ=50000000,BAUD=115200)(
    input wire clk,rst_n,rx,output reg [7:0] data,
    output reg valid,frame_error
);
    localparam integer DIV=CLOCK_HZ/BAUD;
    reg [1:0] sync;reg [1:0] state;reg [31:0] count;
    reg [2:0] bit_index;reg [7:0] shift;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin sync<=3;state<=0;count<=0;bit_index<=0;shift<=0;data<=0;valid<=0;frame_error<=0;end
        else begin
            sync<={sync[0],rx};valid<=0;frame_error<=0;
            case(state)
                0:if(!sync[1])begin state<=1;count<=DIV/2-1;end
                1:if(count!=0)count<=count-1'b1;
                    else if(sync[1])state<=0;
                    else begin state<=2;count<=DIV-1;bit_index<=0;end
                2:if(count!=0)count<=count-1'b1;
                    else begin shift[bit_index]<=sync[1];count<=DIV-1;
                        if(bit_index==7)state<=3;else bit_index<=bit_index+1'b1;end
                3:if(count!=0)count<=count-1'b1;
                    else begin state<=0;if(sync[1])begin data<=shift;valid<=1;end else frame_error<=1;end
            endcase
        end
    end
endmodule
