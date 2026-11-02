// v5 sensor chain: pot -> wrapped theta, encoder -> arm angle, and 8 ms
// windowed velocities with one-pole smoothing. Replaces the 1 ms
// differentiation of v2 whose 5.59 rad/s/count quantization drove the
// derivative term to full-scale chatter.
// ANGLE_REVERSE=1 on hardware (matches the real pot direction); the
// plant model used in simulation already folds the reversal into its
// raw mapping, so the integer reference runs the same RTL with 0.
module c_sensor #(parameter ANGLE_REVERSE=0, WINDOW=8)
 (input clk,rst_n,sample_en,input [11:0] raw12,up12,
 input signed [31:0] count,input reset_history,
 output wire mature,output reg ready,valid,output reg signed [31:0] th,vth_w,vth_f,a,va_w,va_f);
 assign mature=(hist_valid>=WINDOW);
 reg [3:0] step;
 reg signed [13:0] d;
 reg signed [17:0] count_r;
 reg signed [63:0] th_product,a_product;
 reg signed [31:0] th_new,a_new;
 reg signed [31:0] th_hist[0:7];
 reg signed [31:0] a_hist[0:7];
 reg [3:0] hist_valid;
 reg valid_r;
 reg signed [31:0] dth8,da8,dth8_x,da8_x;
 reg signed [63:0] wprod_t,wprod_a;
 reg signed [47:0] ft,fa;
 reg signed [31:0] dvf_t,dvf_a;
 integer i;
 wire signed [13:0] delta=$signed({2'b0,up12})-$signed({2'b0,raw12});
 wire signed [17:0] count18=(count>131071)?18'sd131071:((count< -131072)?-18'sd131072:count[17:0]);
 // window diff against oldest entry, with half-turn wrap for theta
 // Apply the configured sensor direction before both the velocity
 // difference and the history write.  The old code reversed only the
 // published theta while subtracting an unreversed current sample from a
 // reversed history.  With ANGLE_REVERSE=1 this turned a small movement
 // into approximately -(theta_now+theta_old), corrupting the 8 ms speed
 // used by both swing-up and capture.
 wire signed [31:0] th_eff=ANGLE_REVERSE ? -th_new : th_new;
 wire signed [31:0] wd_t0=th_eff-$signed(th_hist[WINDOW-1]);
 wire signed [31:0] wd_t=(wd_t0>205887)?wd_t0-32'sd411775:((wd_t0< -205888)?wd_t0+32'sd411775:wd_t0);
 wire signed [31:0] wd_a=a_new-$signed(a_hist[WINDOW-1]);
 // Constant multiplies written as exact shift-add. All 20 DSPs are spent
 // elsewhere, so "x*C" fell back to a LUT carry chain (~28 ns, over the
 // 24.1 ns requirement). These decompositions are bit-identical:
 //   93827 = 2^16+2^14+2^13+2^11+2^10+2^9+2^7+2^1+2^0  (|d|<=8192  -> 31-bit)
 //   98014 = 2^16+2^15-2^8-2^5-2^1                     (|count|<=131072 -> 35-bit)
 wire signed [31:0] d32 = $signed({{18{d[13]}},d});
 wire signed [31:0] tp_w = (d32<<<16)+(d32<<<14)+(d32<<<13)+(d32<<<11)
                          +(d32<<<10)+(d32<<<9)+(d32<<<7)+(d32<<<1)+d32;
 wire signed [39:0] c40 = $signed(count_r);
 wire signed [39:0] ap_w = (c40<<<16)+(c40<<<15)-(c40<<<8)-(c40<<<5)-(c40<<<1);
 // Window-diff velocity scale and the beta-filter gain, also exact
 // shift-add so they release their DSPs to c_mul32 (which needs one more
 // MULT18X18 for its 4th 9x17 partial product):
 //   125  = 2^7-2^2+2^0
 //   9830 = 2^13+2^10+2^9+2^6+2^5+2^2+2^1
 wire signed [63:0] dt64 = $signed(dth8_x);
 wire signed [63:0] da64 = $signed(da8_x);
 wire signed [63:0] wp_t = (dt64<<<7)-(dt64<<<2)+dt64;
 wire signed [63:0] wp_a = (da64<<<7)-(da64<<<2)+da64;
 wire signed [47:0] dvt48 = $signed(dvf_t);
 wire signed [47:0] dva48 = $signed(dvf_a);
 wire signed [47:0] ft_w = (dvt48<<<13)+(dvt48<<<10)+(dvt48<<<9)+(dvt48<<<6)
                          +(dvt48<<<5)+(dvt48<<<2)+(dvt48<<<1);
 wire signed [47:0] fa_w = (dva48<<<13)+(dva48<<<10)+(dva48<<<9)+(dva48<<<6)
                          +(dva48<<<5)+(dva48<<<2)+(dva48<<<1);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   step<=0;ready<=0;valid<=0;th<=0;vth_w<=0;vth_f<=0;a<=0;va_w<=0;va_f<=0;
   d<=0;count_r<=0;th_product<=0;a_product<=0;th_new<=0;a_new<=0;hist_valid<=0;
   dth8<=0;da8<=0;dth8_x<=0;da8_x<=0;wprod_t<=0;wprod_a<=0;ft<=0;fa<=0;dvf_t<=0;dvf_a<=0;valid_r<=0;
   for(i=0;i<8;i=i+1) begin th_hist[i]<=0; a_hist[i]<=0; end
  end else if(reset_history) begin
   step<=0;ready<=0;vth_w<=0;vth_f<=0;va_w<=0;va_f<=0;hist_valid<=0;
  end else begin
   ready<=0;
   case(step)
    0: if(sample_en) begin
      d<=(delta>2247)?delta-14'sd4494:((delta< -2247)?delta+14'sd4494:delta);
      count_r<=count18;
      valid_r<=(raw12>=16 && raw12<=4076);
      if(!(raw12>=16 && raw12<=4076)) begin
        // invalid: clear velocity state and history fill (ref parity)
        vth_w<=0;vth_f<=0;va_w<=0;va_f<=0;hist_valid<=0;ready<=1;valid<=0;
      end else step<=1;
    end
    1: begin th_product<=tp_w; a_product<=ap_w; step<=2; end
    2: begin th_new<=th_product>>>10; a_new<=a_product>>>8; step<=3; end
    3: begin
      dth8<=wd_t; da8<=wd_a; step<=4;
    end
    4: begin
      dth8_x<=(hist_valid>=WINDOW)?dth8:32'sd0;
      da8_x<=(hist_valid>=WINDOW)?da8:32'sd0;
      step<=5;
    end
    5: begin
      wprod_t<=(hist_valid>=1)?wp_t:64'sd0;
      wprod_a<=(hist_valid>=1)?wp_a:64'sd0;
      step<=6;
    end
    // clip to +/-35 rad/s (2293760 Q16)
    6: begin
      vth_w<=(wprod_t>64'sd2293760)?32'sd2293760:((wprod_t< -64'sd2293760)?-32'sd2293760:wprod_t[31:0]);
      va_w<=(wprod_a>64'sd2293760)?32'sd2293760:((wprod_a< -64'sd2293760)?-32'sd2293760:wprod_a[31:0]);
      step<=7;
    end
    7: begin
      dvf_t<=vth_w-vth_f; dvf_a<=va_w-va_f; step<=8;
    end
    8: begin
      ft<=ft_w; fa<=fa_w; step<=9;
    end
    9: begin
      // beta filter: v += (raw-v)*9830>>16 (only after >=1 valid sample)
      if(hist_valid>=1) begin
        vth_f<=vth_f+(ft>>>16);
        va_f<=va_f+(fa>>>16);
      end else begin
        vth_f<=0; va_f<=0;
      end
      th<=th_eff;
      a<=a_new;
      valid<=1;
      for(i=0;i<7;i=i+1) begin th_hist[i+1]<=th_hist[i]; a_hist[i+1]<=a_hist[i]; end
      th_hist[0]<=th_eff;
      a_hist[0]<=a_new;
      if(hist_valid<WINDOW) hist_valid<=hist_valid+1'b1;
      ready<=1; step<=0;
    end
   endcase
  end
 end
endmodule
