// J280_085 sensor/pump/output adaptation. IO and the balance PID remain local.
// Gains and units in c_sensor/c_swing/c_output are preserved from the supplied source.
module swing_reference_bridge(
 input wire clk,rst_n,enable,update_ms,input wire [10:0] angle_x2,
 input wire [9:0] upright_adc,input wire signed [31:0] relative_position,
 input wire [223:0] config_words,
 output wire qualified,output wire signed [15:0] command,pole_speed_q8,
 output reg capture_request,output reg [1:0] fault_kind,
 output wire [3:0] action,output reg [3:0] diagnostic
);
 wire ready,valid,mature;wire signed [31:0] th,vw,vf,a,aw,af;
 wire [11:0] raw12={angle_x2,1'b0},up12={upright_adc,2'b0};
 wire rail=angle_x2<=8 || angle_x2>=2038;
 c_sensor #(.ANGLE_REVERSE(1),.WINDOW(8)) sensor(.clk(clk),.rst_n(rst_n),
  .sample_en(update_ms && enable),.raw12(raw12),.up12(up12),.count(relative_position),
  .reset_history(!enable),.ready(ready),.valid(valid),.mature(mature),
  .th(th),.vth_w(vw),.vth_f(vf),.a(a),.va_w(aw),.va_f(af));
 assign qualified=enable && valid && mature && !rail && fault_kind==0;
 wire signed [31:0] u;wire pump_done,pump_over,energy_enough,soft_limited;
 wire signed [47:0] soft_product=$signed({1'b0,config_words[7*16+:16]})*32'sd98014;
 reg signed [31:0] soft_q16;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)soft_q16<=0;
  else soft_q16<=soft_product>>>8;
 end
 c_swing pump(.clk(clk),.rst_n(rst_n),.sample_en(ready && qualified),.enable(qualified),
  .th(th),.vth_f(vf),.a(a),.va_f(af),.arm_soft_limit(soft_q16),
  .u(u),.done(pump_done),.overrun(pump_over),.energy_enough(energy_enough),.soft_limited(soft_limited));
 wire signed [47:0] speed_product=$signed({1'b0,config_words[6*16+:16]})*32'sd38287;
 reg signed [31:0] speed_cap;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)speed_cap<=0;
  else speed_cap<=speed_product[31:0];
 end
 wire same_direction=(u>0 && af>0)||(u<0 && af<0);
 wire speed_limited=same_direction && ((af>speed_cap)||(af< -speed_cap));
 wire signed [31:0] request_u=speed_limited ? 32'sd0 : u;
 reg [15:0] duty_cap_value;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)duty_cap_value<=16'd1000;
  else duty_cap_value<=config_words[3*16+:16]/16'd5;
 end
 wire signed [15:0] duty;wire output_done;wire signed [31:0] effective;
 c_output #(.RATE_LIMIT(100),.COMP_D(200),.DUTY_CAP(1000),.RUNTIME_CAP(1)) output_stage(
  .clk(clk),.rst_n(rst_n),.go(pump_done && qualified),.enable(qualified),.clear(!qualified),
  .u(request_u),.duty_cap(duty_cap_value),.done(output_done),.duty(duty),.veff(effective),
  .trace_stage(),.trace_clip(),.trace_rate());
 assign command=qualified ? $signed(duty)*16'sd5 : 16'sd0;
 wire signed [31:0] speed_q8_full=vf>>>8;
 assign pole_speed_q8=speed_q8_full[15:0];
 wire signed [31:0] raw_error=$signed({1'b0,angle_x2})-$signed({1'b0,upright_adc,1'b0});
 wire [31:0] abs_error=raw_error[31] ? -raw_error : raw_error;
 wire in_window=abs_error<=config_words[13*16+:16]*2;
 wire candidate=qualified && in_window && vf<196608 && vf> -196608 &&
  vw<196608 && vw> -196608 && aw<163840 && aw> -163840;
 reg [1:0] confirm_count;reg [7:0] bad_ms;
 reg [3:0] action_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)action_q<=0;
  else action_q<=!qualified ? 4'd9 : speed_limited ? 4'd14 : soft_limited ? 4'd7 : energy_enough ? 4'd13 : 4'd12;
 end
 assign action=action_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin capture_request<=0;fault_kind<=0;confirm_count<=0;bad_ms<=0;diagnostic<=0;end
  else if(!enable)begin capture_request<=0;fault_kind<=0;confirm_count<=0;bad_ms<=0;diagnostic<=0;end
  else if(pump_over)begin fault_kind<=2;capture_request<=0;end
  else begin
   if(rail || !valid || !mature)begin capture_request<=0;confirm_count<=0;end
   if(ready)begin
    if(!valid)begin
     confirm_count<=0;capture_request<=0;diagnostic<=6;
     if(bad_ms>=254)fault_kind<=1;else bad_ms<=bad_ms+1'b1;
    end else begin
     bad_ms<=0;
     if(!candidate || !config_words[9*16])begin confirm_count<=0;capture_request<=0;end
     else if(confirm_count>=2)capture_request<=1;else confirm_count<=confirm_count+1'b1;
     diagnostic<=!mature ? 4'd11 : !in_window ? 4'd0 :
       vf>=196608 || vf<= -196608 || vw>=196608 || vw<= -196608 ? 4'd12 :
       aw>=163840 || aw<= -163840 ? 4'd13 : 4'd14;
    end
   end
  end
 end
endmodule
