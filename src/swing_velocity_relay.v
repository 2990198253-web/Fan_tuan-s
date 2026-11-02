// Independent implementation of a pole-velocity-sign swing-up relay.
// Method reference: chehongshu/Inverted_Pendulum_System, TIMER/timer.c.
// update is 5 ms; arm_velocity remains counts/10 ms. No energy proxy,
// normal position servo, velocity target tracking or fixed inter-kick wait.
module swing_velocity_relay #(
    parameter integer ANGLE_PERIOD_ADC=1060,RETURN_POLARITY=0,DRIVE_POLARITY=0,
    parameter integer DELTA_THRESHOLD_X2=2,MAX_DELTA_X2=80,HOLD_MS=40,INVALID_MS=100
)(
    input wire clk,rst_n,enable,update,
    input wire [223:0] config_words,input wire [10:0] angle_x2,
    input wire signed [32:0] displacement,arm_velocity,
    input wire capture_zone,
    output reg signed [15:0] command,delta_x2,
    output reg [3:0] action,cut_reason,
    output reg [15:0] step_ms,last_accel_ms,pulse_index,
    output reg done
);
    reg initialized,seed_active,returning,finishing,history_valid;
    reg direction_positive,pending_positive;
    reg [10:0] previous_angle;
    reg signed [32:0] seed_origin;
    reg [15:0] confirm_count,motion_age_ms,quiet_ms,invalid_ms,brake_age_ms;
    wire signed [15:0] cap=$signed(config_words[3*16+:16]);
    wire signed [15:0] brake_cap=$signed(config_words[4*16+:16]);
    wire signed [15:0] return_cap=$signed(config_words[12*16+:16]);
    wire [15:0] speed_cap=config_words[6*16+:16],soft=config_words[7*16+:16];
    wire [15:0] seed_ms=config_words[0*16+:16]>80 ? 16'd80 : config_words[0*16+:16];
    wire signed [31:0] raw_delta=$signed({1'b0,angle_x2})-$signed({1'b0,previous_angle});
    wire signed [31:0] wrapped_delta=raw_delta>ANGLE_PERIOD_ADC ? raw_delta-ANGLE_PERIOD_ADC*2 :
        raw_delta < -ANGLE_PERIOD_ADC ? raw_delta+ANGLE_PERIOD_ADC*2 : raw_delta;
    wire [31:0] abs_delta=wrapped_delta[31] ? -wrapped_delta : wrapped_delta;
    wire sample_good=angle_x2>8 && angle_x2<2038 && (!history_valid || abs_delta<=MAX_DELTA_X2);
    wire motion_valid=sample_good && history_valid && abs_delta>=DELTA_THRESHOLD_X2;
    // ADC increases as the reference project's converted pole angle decreases.
    // Wiring polarity remains explicitly adjustable; never infer it from names.
    wire requested_positive=(!wrapped_delta[31]) ^ (DRIVE_POLARITY!=0) ^ config_words[9*16+2];
    wire direction_accept=motion_valid && requested_positive!=direction_positive &&
        (config_words[10*16+:16]<=1 ||
        (pending_positive==requested_positive && confirm_count>=config_words[10*16+:16]-1));
    wire selected_positive=direction_accept ? requested_positive : direction_positive;
    wire [32:0] abs_v=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire [32:0] abs_p=displacement[32] ? -displacement : displacement;
    wire signed [33:0] distance=$signed({displacement[32],displacement})-$signed({seed_origin[32],seed_origin});
    wire [33:0] abs_distance=distance[33] ? -distance : distance;
    wire outward=displacement==0 || displacement[32]==arm_velocity[32];
    wire signed [63:0] prediction=$signed(displacement)+$signed(arm_velocity)*2;
    wire [63:0] abs_prediction=prediction[63] ? -prediction : prediction;
    wire guard=abs_p>=soft || (outward && abs_prediction>=soft);
    wire brake_positive=arm_velocity[32] ^ (RETURN_POLARITY!=0);
    wire signed [15:0] braking=abs_v<=1 ? 16'sd0 : brake_positive ? brake_cap : -brake_cap;
    wire signed [63:0] return_normal=-$signed(displacement)*16-$signed(arm_velocity)*200;
    wire signed [63:0] return_physical=RETURN_POLARITY!=0 ? -return_normal : return_normal;
    wire signed [15:0] return_command=return_physical>return_cap ? return_cap :
        return_physical < -return_cap ? -return_cap : return_physical[15:0];
    wire normal_positive=selected_positive ^ (RETURN_POLARITY!=0);
    wire pushes_speed=arm_velocity==0 || normal_positive!=arm_velocity[32];
    wire [32:0] headroom=abs_v>=speed_cap ? 33'd0 : {17'd0,speed_cap}-abs_v;
    wire [48:0] speed_product={1'b0,cap}*headroom;
    wire signed [15:0] amplitude=pushes_speed && headroom<4 ? $signed(speed_product[17:2]) : cap;
    wire hold_valid=motion_valid || motion_age_ms<HOLD_MS;
    wire signed [15:0] relay_command=!hold_valid ? 16'sd0 : selected_positive ? amplitude : -amplitude;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            initialized<=0;seed_active<=0;returning<=0;finishing<=0;history_valid<=0;
            direction_positive<=0;pending_positive<=0;previous_angle<=0;seed_origin<=0;
            confirm_count<=0;motion_age_ms<=0;quiet_ms<=0;invalid_ms<=0;brake_age_ms<=0;
            command<=0;delta_x2<=0;action<=0;cut_reason<=0;step_ms<=0;last_accel_ms<=0;pulse_index<=0;done<=0;
        end else if(!enable)begin
            initialized<=0;seed_active<=0;returning<=0;finishing<=0;history_valid<=0;
            confirm_count<=0;motion_age_ms<=0;quiet_ms<=0;invalid_ms<=0;brake_age_ms<=0;
            command<=0;delta_x2<=0;action<=0;done<=0;step_ms<=0;
        end else if(update)begin
            previous_angle<=angle_x2;history_valid<=sample_good;
            delta_x2<=sample_good && history_valid ? wrapped_delta[15:0] : 16'sd0;
            if(step_ms<65000)step_ms<=step_ms+16'd5;
            if(sample_good)invalid_ms<=0;else if(invalid_ms<1000)invalid_ms<=invalid_ms+16'd5;
            if(motion_valid)begin motion_age_ms<=0;quiet_ms<=0;end
            else begin
                if(motion_age_ms<1000)motion_age_ms<=motion_age_ms+16'd5;
                if(quiet_ms<1000)quiet_ms<=quiet_ms+16'd5;
            end
            if(done)begin command<=0;action<=10;end
            else if(!sample_good && invalid_ms+16'd5>=INVALID_MS)begin
                command<=0;done<=1;cut_reason<=6;action<=9;
            end else if(!sample_good || (!history_valid && initialized))begin
                // Invalid angle cannot fall through to recenter torque.
                command<=0;action<=9;cut_reason<=6;confirm_count<=0;seed_active<=0;
            end else if(guard || returning || abs_v>=24)begin
                returning<=1;seed_active<=0;confirm_count<=0;
                command<=(outward && abs_v>1)||abs_v>=24 ? braking : return_command;
                action<=abs_v>=24 ? 4'd6 : 4'd7;cut_reason<=abs_v>=24 ? 4'd9 : 4'd4;
                if((outward && abs_v>1)||abs_v>=24)begin
                    if(brake_age_ms+16'd5>=config_words[1*16+:16])begin command<=0;done<=1;cut_reason<=7;end
                    else brake_age_ms<=brake_age_ms+16'd5;
                end else brake_age_ms<=0;
                if(!guard && abs_p<=60 && abs_v<=1)begin returning<=0;step_ms<=0;brake_age_ms<=0;end
            end else if(!sample_good || (capture_zone && config_words[9*16]))begin
                command<=braking;action<=sample_good ? 4'd8 : 4'd9;cut_reason<=sample_good ? 4'd5 : 4'd6;
                seed_active<=0;confirm_count<=0;
            end else if(!initialized)begin
                initialized<=1;seed_active<=1;seed_origin<=displacement;step_ms<=0;
                direction_positive<=(DRIVE_POLARITY==0)^config_words[9*16+2];pulse_index<=1;
                command<=((DRIVE_POLARITY==0)^config_words[9*16+2]) ? cap : -cap;
                action<=15;cut_reason<=0;last_accel_ms<=0;
            end else if(finishing || (!seed_active && pulse_index>=config_words[8*16+:16]))begin
                finishing<=1;command<=braking;action<=4;
                if(abs_v<=1)begin command<=0;done<=1;action<=10;end
                else if(brake_age_ms+16'd5>=config_words[1*16+:16])begin command<=0;done<=1;cut_reason<=7;end
                else brake_age_ms<=brake_age_ms+16'd5;
            end else if(seed_active)begin
                if(step_ms+16'd5>=seed_ms || abs_distance>=config_words[5*16+:16] || abs_v>=speed_cap)begin
                    seed_active<=0;last_accel_ms<=step_ms+16'd5;step_ms<=0;command<=0;cut_reason<=10;action<=5;
                end else begin command<=direction_positive ? cap : -cap;action<=15;end
            end else begin
                command<=relay_command;cut_reason<=0;
                action<=!hold_valid ? 4'd5 : pushes_speed && headroom<4 ? 4'd14 : selected_positive ? 4'd12 : 4'd13;
                if(direction_accept)begin
                    direction_positive<=requested_positive;confirm_count<=0;step_ms<=0;pulse_index<=pulse_index+1'b1;
                end else if(!motion_valid || requested_positive==direction_positive)confirm_count<=0;
                else if(confirm_count==0 || pending_positive!=requested_positive)begin pending_positive<=requested_positive;confirm_count<=1;end
                else confirm_count<=confirm_count+1'b1;
                // Only a genuinely quiet mechanism may be re-seeded; this is
                // not the normal pumping cadence and never bypasses protection.
                if(!motion_valid && quiet_ms>=300 && abs_v<=1)begin
                    seed_active<=1;seed_origin<=displacement;direction_positive<=!direction_positive;
                    step_ms<=0;pulse_index<=pulse_index+1'b1;quiet_ms<=0;
                end
            end
        end
    end
endmodule
