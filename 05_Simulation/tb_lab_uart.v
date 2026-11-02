`timescale 1ns/1ps
module tb_lab_uart;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,rx=1;wire [7:0] data;wire valid,error;
    integer received=0,errors=0;reg [7:0] bytes[0:3];
    lab_uart_rx #(.CLOCK_HZ(160000),.BAUD(10000)) dut(.clk(clk),.rst_n(rst),.rx(rx),.data(data),.valid(valid),.frame_error(error));
    always @(posedge clk)begin if(valid)begin bytes[received]=data;received=received+1;end if(error)errors=errors+1;end
    task bit_send;input b;begin rx=b;repeat(16)@(negedge clk);end endtask
    task send;input [7:0] b;input stopbit;integer i;begin bit_send(0);for(i=0;i<8;i=i+1)bit_send(b[i]);bit_send(stopbit);end endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(4)@(negedge clk);
        rx=0;repeat(3)@(negedge clk);rx=1;repeat(25)@(negedge clk);
        if(received!=0)$fatal(1,"False start accepted");
        send(8'hA5,1);send(8'h00,1);send(8'hFF,1);repeat(20)@(negedge clk);
        if(received!=3||bytes[0]!=8'hA5||bytes[1]!=0||bytes[2]!=255)$fatal(1,"8N1 sampling");
        send(8'h55,0);rx=1;repeat(25)@(negedge clk);
        if(errors<1||received!=3)$fatal(1,"Bad stop bit accepted");
        $display("SWING_LAB_UART_PASS false_start contiguous_bytes stop_rejection");$finish;
    end
    initial begin #1000000;$fatal(1,"UART timeout");end
endmodule
