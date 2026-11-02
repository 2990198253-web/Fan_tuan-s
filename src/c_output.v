// M1: P_COMP_D=0 => linear voltage-to-duty mapping, bounded by P_DUTY_CAP.
// Nominal voltage uses final duty after slew/clipping. No inferred deadzone.
// v5 shared output stage: volts -> duty with a continuous piecewise-linear
// deadband inverse (replaces the v2 +/-241 relay), clip, slew limiter
// (replaces the 500 Hz forced dither), and the effective-voltage report
// the observer predicts with (what the motor actually receives after the
// plant deadband, estimated at COMP_D). Mirrors ref_v5.OutputStage.step
// operation for operation; u must be held stable while the FSM is busy.
// Timing notes: every constant multiply is written as an explicit
// shift-add (a plain "x*C" gets mapped to a DSP, and all 20 DSPs are
// already spent elsewhere), and the magnitude path is narrowed to the
// widths it can actually take -- |u|<=P_VMAX -> mv<=12000 (14 bits) ->
// |duty|<=1000 (11 bits) -- so the LUT carry chains stay inside 20 ns.
// COMP_D / DUTY_CAP default to the v5 params (balance, COMP_D=0, cap=500).
// The swing-up path instantiates this with COMP_D=200 and DUTY_CAP=1000 to
// reproduce the proven v6 softlimit120 swing output (fixed 2.4 V deadband
// inverse, full-scale duty).
module c_output #(parameter integer RATE_LIMIT=100, COMP_D=-1, DUTY_CAP=-1,RUNTIME_CAP=0)
 (input clk,rst_n,go,enable,clear,
 input signed [31:0] u,
 input wire [15:0] duty_cap,
 output reg done, output reg signed [15:0] duty, output reg signed [31:0] veff,
 output wire [31:0] trace_stage, output wire trace_clip,trace_rate);
 assign trace_stage={d_clip[15:0],d_knee[15:0]};
 assign trace_clip=(d_clip!=d_knee);
 assign trace_rate=(d_rate!=d_clip);
`include "v5_params.vh"
 localparam signed [31:0] CD  = (COMP_D<0)   ? P_COMP_D  : COMP_D;
 wire signed [31:0] CAP = RUNTIME_CAP!=0 ? $signed({1'b0,duty_cap}) : (DUTY_CAP<0) ? P_DUTY_CAP: DUTY_CAP;
 localparam signed [31:0] DB_Q16 = (CD*P_VMAX)/1000;
 localparam signed [31:0] SLOPE = (P_COMP_EPS+CD)/P_COMP_EPS;
 reg [3:0] step;
 reg sign_neg, sign_v;
 reg signed [31:0] u_r, d_knee, d_clip, d_rate, vfull, mag_d;
 reg [31:0] mv_r;              // |u| scaled to millivolts (pipeline s1)
 reg [31:0] dmag_r;            // |duty| = floor(mv/12) (pipeline s2)
 reg signed [31:0] ds_r;       // signed duty before PWL (pipeline s3)
 reg [29:0] mag, qt;
 reg signed [63:0] prod;
 reg [19:0] q;
 reg signed [15:0] prev_duty_r;
 // pipelined magnitude/duty path (small constant multiplies, shift-add form)
 wire [31:0] u_abs = (u_r<0) ? -u_r : u_r;
 wire [31:0] u_mag = (u_abs > P_VMAX) ? P_VMAX : u_abs;
 wire [31:0] mv_w = ((u_mag<<10)-(u_mag<<4)-(u_mag<<3))>>16;   // *1000, 0..12000
wire [13:0] mvr = mv_r[13:0];                                 // mv_r <= 12000
// NOTE: the product must be given its own 27-bit context. Written as
// "(mvr*13'd5461)>>16" the Verilog width rule sizes the multiply to
// max(14,13,11)=14 bits, truncating 5461*12000 (6.5e7) before the shift
// and making d0 collapse to 0.
wire [26:0] d0p_w = mvr*13'd5461;
 reg [26:0] d0p_r;              // registered: this multiply was the 24 ns path
 wire [26:0] d0p = d0p_r;                               // <= 6.5532e7
wire [10:0] d0 = d0p[26:16];                                  // <= floor(mv/12)
 wire [10:0] d0p1 = d0+11'd1;
 wire [14:0] d0x12 = (d0p1<<3)+(d0p1<<2);                      // (d0+1)*12 <= 12012
 wire [31:0] dmag_w = (d0x12 <= {1'b0,mvr}) ? {21'b0,d0p1} : {21'b0,d0}; // exact floor
 wire signed [31:0] d_s = sign_neg ? -$signed(dmag_r) : $signed(dmag_r);
 wire in_knee = (d_s <= P_COMP_EPS) && (d_s >= -P_COMP_EPS);
 wire signed [31:0] d_pwl = in_knee ? (d_s*SLOPE) :
                            (d_s > 0) ? (d_s + CD) : (d_s - CD);
 wire signed [31:0] d_cl = (d_pwl > CAP) ? CAP :
                           (d_pwl < -CAP) ? -CAP : d_pwl;
 wire signed [31:0] hi = {{16{prev_duty_r[15]}},prev_duty_r} + RATE_LIMIT;
 wire signed [31:0] lo = {{16{prev_duty_r[15]}},prev_duty_r} - RATE_LIMIT;
 wire signed [31:0] d_rt = (d_cl > hi) ? hi : ((d_cl < lo) ? lo : d_cl);
 wire signed [31:0] vfull_w = sign_v ? -$signed({12'b0,q}) : $signed({12'b0,q});
wire signed [31:0] veff_w = (vfull_w > DB_Q16) ? (vfull_w - DB_Q16) :
                            (vfull_w < -DB_Q16) ? (vfull_w + DB_Q16) : 32'sd0;
 // *4294968 written as an exact shift-add (it is the only DSP-sized
 // constant left here and would otherwise claim two DSPs):
 //   4294968 = 2^22+2^17-2^15+2^11+2^8+2^6-2^3
 // mag = |duty|*P_VMAX <= 1000*786432 = 7.864e8, so the product needs
 // 52 bits (3.377e15 < 2^52) -- prod[51:32] is the reciprocal quotient.
 wire signed [51:0] mg52 = $signed({22'b0,mag});
 wire signed [51:0] prod_w = (mg52<<<22)+(mg52<<<17)-(mg52<<<15)+(mg52<<<11)
                            +(mg52<<<8)+(mg52<<<6)-(mg52<<<3);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   step<=0;done<=0;duty<=0;veff<=0;sign_neg<=0;sign_v<=0;u_r<=0;d_knee<=0;mv_r<=0;dmag_r<=0;ds_r<=0;d0p_r<=0;
   d_clip<=0;d_rate<=0;vfull<=0;mag_d<=0;mag<=0;qt<=0;prod<=0;q<=0;prev_duty_r<=0;
  end else if(clear) begin
    step<=0;done<=0;duty<=0;veff<=0;prev_duty_r<=0;
  end else begin
   done<=0;
   case(step)
    0: if(go) begin
      u_r<=u; sign_neg<=(u<0);
      if(!enable) begin duty<=0; veff<=0; prev_duty_r<=0; done<=1; end
      else step<=1;
    end
    1: begin mv_r<=mv_w; step<=2; end
    2: begin d0p_r<=d0p_w; step<=13; end
    13: begin dmag_r<=dmag_w; step<=3; end
    3: begin ds_r<=d_s; d_knee<=d_pwl; step<=4; end
    4: begin d_clip<=d_cl; step<=5; end
    5: begin
      d_rate<=d_rt;
      prev_duty_r<=d_rt[15:0];
      duty<=d_rt[15:0];
      mag_d<=(d_rt<0)?(-d_rt):d_rt;   // |duty| <= 1000
      sign_v<=(d_rt<0);               // veff follows the FINAL duty sign
      step<=6;
    end
    6: begin mag<=(mag_d<<19)+(mag_d<<18); step<=7; end  // mag_d*P_VMAX, <=7.86e8
    7: begin prod<=prod_w; step<=8; end
    8: begin q<=prod[51:32]; step<=9; end
    9: begin qt<=({10'b0,q}<<10)-({10'b0,q}<<4)-({10'b0,q}<<3); step<=10; end  // q*1000
    10: begin if(qt>mag) q<=q-1; step<=11; end      // exact floor(|duty|*786432/1000)
    11: begin vfull<=vfull_w; step<=12; end
    12: begin veff<=veff_w; done<=1; step<=0; end
   endcase
  end
 end
endmodule
