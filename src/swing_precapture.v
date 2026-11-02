// Local upright damping before handing off to the unchanged balance loop.
// The observer runs throughout relay swing-up: no cold-start derivative at entry.
// delta_x2/5ms numerically equals ADC/10ms for constant pole velocity.
module swing_precapture #(
    parameter integer UPRIGHT_ADC=755,BALANCE_POLARITY=0,RETURN_POLARITY=0,
    HARD_TRAVEL=240,P_GAIN_X2=110,P_LIMIT=900,D_GAIN=200,BOUNDARY_MARGIN=24
)(
    input wire clk,rst_n,enable,active,update,
    input wire [10:0] angle_x2,
    input wire signed [32:0] displacement,arm_velocity,
    input wire [15:0] command_cap,
    output wire observation_valid,output wire boundary_limited,
    output wire signed [15:0] command,
    output reg signed [15:0] pole_delta_x2
);
    reg [10:0] previous;
    reg history;
    reg [1:0] good_count;
    reg signed [15:0] damp_hold;
    reg signed [12:0] previous_delta;
    wire signed [12:0] delta=$signed({1'b0,angle_x2})-$signed({1'b0,previous});
    wire [12:0] abs_delta=delta[12] ? -delta : delta;
    wire rail=angle_x2<=8 || angle_x2>=2038;
    wire sample_good=!rail && (!history || abs_delta<=80);
    // Also gate a newly arriving bad ADC asynchronously to the 5ms update.
    assign observation_valid=enable && history && good_count>=3 && sample_good;
    wire signed [31:0] error=$signed({1'b0,angle_x2})-UPRIGHT_ADC*2;
    wire signed [31:0] p_product=error*P_GAIN_X2;
    wire signed [31:0] p_term=p_product>P_LIMIT ? P_LIMIT : p_product < -P_LIMIT ? -P_LIMIT : p_product;
    // Two adjacent 5ms differences provide a 10ms moving derivative. This
    // avoids alternating D/zero on quantized or 10ms replayed observations.
    wire signed [31:0] delta_sum=$signed(delta)+$signed(previous_delta);
    wire signed [31:0] d_term=delta_sum*(D_GAIN/2);
    wire signed [31:0] sum=p_term+d_term;
    wire signed [31:0] oriented=BALANCE_POLARITY!=0 ? -sum : sum;
    wire signed [31:0] cap=$signed({1'b0,command_cap});
    wire signed [15:0] limited=oriented>cap ? command_cap : oriented < -cap ? -$signed(command_cap) : oriented[15:0];
    wire [32:0] abs_p=displacement[32] ? -displacement : displacement;
    wire [32:0] abs_v=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire signed [63:0] prediction=$signed(displacement)+$signed(arm_velocity)*2;
    wire [63:0] abs_prediction=prediction[63] ? -prediction : prediction;
    wire normalized_positive=(!damp_hold[15]) ^ (RETURN_POLARITY!=0);
    wire requested_out=damp_hold!=0 && displacement!=0 && normalized_positive!=displacement[32];
    wire moving_out=arm_velocity!=0 && (displacement==0 || displacement[32]==arm_velocity[32]);
    assign boundary_limited=active && ((abs_p>=HARD_TRAVEL-BOUNDARY_MARGIN && (requested_out || moving_out)) ||
        (moving_out && abs_prediction>=HARD_TRAVEL-12) || abs_v>=24);
    wire signed [15:0] brake=abs_v<=1 || (!moving_out && abs_v<24) ? 16'sd0 : (arm_velocity[32] ^ (RETURN_POLARITY!=0)) ?
        $signed(command_cap) : -$signed(command_cap);
    assign command=active && observation_valid ? (boundary_limited ? brake : damp_hold) : 16'sd0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin previous<=0;history<=0;good_count<=0;damp_hold<=0;pole_delta_x2<=0;previous_delta<=0;end
        else if(!enable)begin history<=0;good_count<=0;damp_hold<=0;pole_delta_x2<=0;previous_delta<=0;end
        else if(update)begin
            previous<=angle_x2;history<=sample_good;
            if(!sample_good)begin good_count<=0;damp_hold<=0;pole_delta_x2<=0;previous_delta<=0;end
            else if(!history)begin good_count<=1;damp_hold<=0;pole_delta_x2<=0;previous_delta<=0;end
            else begin
                if(good_count<3)good_count<=good_count+1'b1;
                pole_delta_x2<=delta;
                previous_delta<=delta;
                damp_hold<=limited;
            end
        end
    end
endmodule
