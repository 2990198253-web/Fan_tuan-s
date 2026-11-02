// A calibration request never enables the motor or queues a later start.
module lab_zero_calibrator #(parameter integer DEFAULT_ADC=755)(
 input wire clk,rst_n,enable,idle,update_ms,request,
 input wire [10:0] angle_x2,input wire signed [31:0] position,
 output reg busy,valid,failed,output reg [9:0] zero_adc,output reg [12:0] revision
);
 reg [7:0] count;reg [18:0] sum;reg [10:0] lo,hi;reg bad;
 reg signed [31:0] origin;
 wire sample_bad=angle_x2<1300 || angle_x2>1800 ||
     angle_x2+6<hi || angle_x2>lo+6 || $signed(position)>$signed(origin)+1 || $signed(position)<$signed(origin)-1;
 wire [19:0] total={1'b0,sum}+angle_x2;
 wire [19:0] rounded_total=total+20'd256;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;valid<=0;failed<=0;zero_adc<=DEFAULT_ADC;revision<=0;count<=0;sum<=0;lo<=0;hi<=0;bad<=0;origin<=0;end
  else if(!enable)begin busy<=0;valid<=0;failed<=0;end
  else if(!idle && busy)begin busy<=0;valid<=0;failed<=1;end
  else if(request && idle && !busy)begin
   busy<=1;valid<=0;failed<=0;count<=0;sum<=0;lo<=angle_x2;hi<=angle_x2;origin<=position;bad<=0;
  end else if(busy && update_ms)begin
   sum<=total[18:0];count<=count+1'b1;
   if(angle_x2<lo)lo<=angle_x2;if(angle_x2>hi)hi<=angle_x2;
   if(sample_bad)bad<=1;
   if(count==255)begin
    busy<=0;
    if(bad || sample_bad)begin failed<=1;valid<=0;end
    else begin zero_adc<=rounded_total[18:9];valid<=1;failed<=0;revision<=revision+1'b1;end
   end
  end
 end
endmodule
