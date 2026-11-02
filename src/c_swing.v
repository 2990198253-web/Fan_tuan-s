// v5 swing-up: same proven v2 energy pump law, re-based on windowed
// velocities, with an 8% target-energy margin so the pendulum crosses
// the top with capturable residual speed, and output in volts (the
// shared output stage in c_control applies comp/slew -- no more relay
// compensation or 500 Hz dither inside the pump).
module c_swing(input clk,rst_n,sample_en,enable,
 input signed [31:0] th,vth_f,a,va_f,
 input signed [31:0] arm_soft_limit,
 output wire energy_enough,soft_limited,
 output reg signed [31:0] u, output reg done,overrun);
`include "v5_params.vh"
 reg [3:0] step;
 reg signed [31:0] t,v,arm,speed,co;
 reg signed [31:0] vv2;
 reg signed [63:0] damping,vel_ff,acc_ff,vv,e_prod;
 reg signed [31:0] v_sq,e_thr;
 reg energetic;
 reg signed [31:0] pump,acc;
 // Relative-arm guard: keep the swing-up pump inside a practical travel
 // envelope around the latched anchor.  The existing 5/2 arm term is too
 // small against the +/-4587520 energy pump, so it cannot stop a full turn.
 // The old build used +90 deg and -120 deg, which gave the two physical
 // half-turns different pump authority.  076 deliberately makes this guard
 // symmetric; the separate control FSM still owns the hard +/-150 deg abort.
 wire signed [31:0] ARM_SOFT_LIMIT = arm_soft_limit;
 wire signed [31:0] ARM_SOFT_NEG = -arm_soft_limit;
 // Continuous arm-velocity damping in the pump output: caps the arm
 // stroke speed so the swing cannot build a runaway oscillation while
 // the pendulum still gains energy (the hard bound in c_control is the
 // final safety net). 0.5 V per rad/s of arm speed.
 localparam signed [31:0] ARM_DAMP_Q8 = 32'sd128;
 wire [31:0] t_abs=(th<0)?-th:th;
 wire [5:0] index=(t_abs>204800)?6'd50:t_abs[17:12];
 wire signed [63:0] vprod=v*v;          // force 64-bit product context
 // The energy-pump direction only needs the SIGN of v*co. Over integers
 // (v*co)>0  <=>  (v>0 && co>0) || (v<0 && co<0), so computing the sign
 // directly is bit-exact and drops a 32x32 MULT36X36 that was the
 // design's critical path (co -> vco -> pump, 19.2 ns of a 19.8 ns budget).
 wire vco_pos = (v>0 && co>0) || (v<0 && co<0);
 // Keep the critical acc_sum path identical to the proven implementation.
 // The guard is applied when the pump command is registered below: once the
 // arm reaches +/-120 deg, an outward energy-pump command is suppressed and
 // the existing 5/2 restoring term brings it back inward.
 wire signed [31:0] pump_base = energetic ? 32'sd0 :
    ((v<=1966 && v>=-1966) ? ((t>163840 || t< -163840)?32'sd327680:32'sd0) :
     (vco_pos?-32'sd4587520:32'sd4587520));
 wire pump_base_outward=(arm>0 && pump_base>0) || (arm<0 && pump_base<0);
 wire signed [31:0] pump_limited =
    ((arm>=ARM_SOFT_LIMIT || arm<=ARM_SOFT_NEG) && pump_base_outward) ? 32'sd0 : pump_base;
 assign energy_enough=energetic;
 assign soft_limited=pump_limited!=pump_base;
 wire signed [63:0] acc_sum=$signed(pump)-(arm*64'sd5>>>1)-(damping>>>16);
 // Exact shift-add forms of the constant multiplies (all 20 DSPs are spent,
 // so "x*C" would otherwise become a LUT carry chain):
 //   39322 = 2^15+2^12+2^11+2^8+2^7+2^4+2^3+2^1
 //   5848  = 2^12+2^10+2^9+2^7+2^6+2^4+2^3
 //   13186 = 2^13+2^12+2^9+2^8+2^7+2^1
 wire signed [39:0] sp40 = $signed(speed);
 wire signed [39:0] damp_w = (sp40<<<15)+(sp40<<<12)+(sp40<<<11)+(sp40<<<8)
                            +(sp40<<<7)+(sp40<<<4)+(sp40<<<3)+(sp40<<<1);
 wire signed [39:0] ac40 = $signed(acc);
 wire signed [39:0] aff_w = (ac40<<<12)+(ac40<<<10)+(ac40<<<9)+(ac40<<<7)
                           +(ac40<<<6)+(ac40<<<4)+(ac40<<<3);
 wire signed [39:0] vff_w = (sp40<<<13)+(sp40<<<12)+(sp40<<<9)+(sp40<<<8)
                           +(sp40<<<7)+(sp40<<<1);
 wire signed [39:0] armd40 = $signed(speed)*$signed(ARM_DAMP_Q8);
 // (65536-co) only needs 18 bits: co is a Q16 cosine in [-65527,65536], so
 // the difference lies in [0,131063]. Writing it as a 32x32 product made
 // this the single near-critical path of the whole design (co -> e_prod,
 // 18.58 ns of a 19.8 ns budget). Narrowing it to 25x18 maps it onto one
 // MULTALU36X18 instead of a MULT36X36 and shortens the path.
 wire signed [17:0] com = 18'sd65536 - $signed(co[17:0]);
 wire signed [24:0] pen25 = {1'b0,P_E_NUM[23:0]};      // 13886816
 wire signed [42:0] eprod_w = pen25*com;               // <= 1.821e12 < 2^41
 function signed [31:0] cos_rom;input [5:0] idx;begin
  case(idx)
     0:cos_rom=32'sd65536;   1:cos_rom=32'sd65408;   2:cos_rom=32'sd65025;
     3:cos_rom=32'sd64387;   4:cos_rom=32'sd63499;   5:cos_rom=32'sd62362;
     6:cos_rom=32'sd60982;   7:cos_rom=32'sd59363;   8:cos_rom=32'sd57513;
     9:cos_rom=32'sd55439;  10:cos_rom=32'sd53147;  11:cos_rom=32'sd50649;
    12:cos_rom=32'sd47952;  13:cos_rom=32'sd45068;  14:cos_rom=32'sd42008;
    15:cos_rom=32'sd38785;  16:cos_rom=32'sd35409;  17:cos_rom=32'sd31896;
    18:cos_rom=32'sd28258;  19:cos_rom=32'sd24509;  20:cos_rom=32'sd20665;
    21:cos_rom=32'sd16740;  22:cos_rom=32'sd12750;  23:cos_rom=32'sd8710;
    24:cos_rom=32'sd4636;   25:cos_rom=32'sd544;    26:cos_rom=-32'sd3551;
    27:cos_rom=-32'sd7631;  28:cos_rom=-32'sd11682; 29:cos_rom=-32'sd15687;
    30:cos_rom=-32'sd19630; 31:cos_rom=-32'sd23497; 32:cos_rom=-32'sd27273;
    33:cos_rom=-32'sd30941; 34:cos_rom=-32'sd34489; 35:cos_rom=-32'sd37903;
    36:cos_rom=-32'sd41168; 37:cos_rom=-32'sd44273; 38:cos_rom=-32'sd47204;
    39:cos_rom=-32'sd49951; 40:cos_rom=-32'sd52504; 41:cos_rom=-32'sd54851;
    42:cos_rom=-32'sd56984; 43:cos_rom=-32'sd58895; 44:cos_rom=-32'sd60575;
    45:cos_rom=-32'sd62019; 46:cos_rom=-32'sd63221; 47:cos_rom=-32'sd64176;
    48:cos_rom=-32'sd64880; 49:cos_rom=-32'sd65331; 50:cos_rom=-32'sd65527;
    default:cos_rom=-32'sd65527;
  endcase
 end endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   step<=0;u<=0;done<=0;overrun<=0;
   t<=0;v<=0;arm<=0;speed<=0;co<=0;vv2<=0;v_sq<=0;e_thr<=0;energetic<=0;
   damping<=0;vel_ff<=0;acc_ff<=0;vv<=0;e_prod<=0;pump<=0;acc<=0;
  end else if(!enable || overrun) begin step<=0;u<=0;done<=0; end
  else if(sample_en && step!=0) begin overrun<=1;step<=0;u<=0;done<=0; end
  else begin
   done<=0;
   case(step)
    0:if(sample_en) begin
      t<=th;v<=vth_f;arm<=a;speed<=va_f;
      co<=cos_rom(index);
      step<=1;
    end
    1:begin
      v_sq<=vprod>>>16;                 // <= 8.03e7, fits 32 bits
      e_prod<=eprod_w;
      damping<=damp_w;
      step<=2;
    end
    2:begin
      e_thr<=e_prod>>>16;               // <= 2.78e7, fits 32 bits
      step<=3;
    end
    3:begin
      energetic<=(v_sq>=e_thr);
      step<=4;
    end
    4:begin
      pump<=pump_limited;
      step<=5;
    end
    5:begin
      acc<=(acc_sum>64'sd4587520)?32'sd4587520:((acc_sum< -64'sd4587520)?-32'sd4587520:acc_sum[31:0]);
      step<=6;
    end
    6:begin acc_ff<=aff_w; vel_ff<=vff_w; step<=7; end
    7:begin
      vv<=(acc_ff>>>16)+(vel_ff>>>16);
      step<=8;
    end
    8:begin
      // subtract arm-velocity damping (opposes the arm's motion)
      vv<=(vv>64'sd786432)?32'sd786432:((vv< -64'sd786432)?-32'sd786432:vv[31:0]);
      step<=9;
    end
    9:begin
      vv2<=vv-$signed(armd40[39:8]);
      step<=10;
    end
   10:begin
      u<=(vv2>64'sd786432)?32'sd786432:((vv2< -64'sd786432)?-32'sd786432:vv2[31:0]);
      done<=1;step<=0;
    end
   endcase
  end
 end
endmodule
