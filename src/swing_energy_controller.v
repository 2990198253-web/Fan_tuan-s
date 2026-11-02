// LAB2 voltage/torque energy-shaping approximation, NOT an identified Furuta
// inverse-dynamics controller. No fixed accelerate/brake/wait pulse cadence.
// The energy proxy and phase come from the measured pole angle/velocity.
module swing_energy_controller #(parameter integer RETURN_POLARITY=0,DRIVE_POLARITY=0)(
    input wire clk,rst_n,enable,update,
    input wire [223:0] config_words,
    input wire signed [32:0] displacement,arm_velocity,
    input wire signed [31:0] energy_error_q8,
    input wire signed [15:0] cosine,
    input wire phase_positive,phase_valid,angle_valid,capture_zone,
    output reg signed [15:0] command,
    output reg [3:0] action,cut_reason,
    output reg [15:0] step_ms,last_accel_ms,pulse_index,
    output reg done
);
    reg initialized,seed_active,seed_positive,phase_latched,pending,returning,finishing;
    reg [15:0] confirm_count,quiet_ms,phase_age_ms,brake_age_ms;
    reg signed [32:0] seed_origin;
    wire signed [15:0] cap=$signed(config_words[3*16+:16]);
    wire signed [15:0] brake_cap=$signed(config_words[4*16+:16]);
    wire signed [15:0] return_cap=$signed(config_words[12*16+:16]);
    wire [15:0] speed_cap=config_words[6*16+:16],soft=config_words[7*16+:16];
    wire [15:0] seed_ms=config_words[0*16+:16]>80 ? 16'd80 : config_words[0*16+:16];
    wire [32:0] abs_v=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire [32:0] abs_p=displacement[32] ? -displacement : displacement;
    wire signed [33:0] seed_distance=$signed({displacement[32],displacement})-$signed({seed_origin[32],seed_origin});
    wire [33:0] abs_seed_distance=seed_distance[33] ? -seed_distance : seed_distance;
    wire signed [63:0] predicted=$signed(displacement)+$signed(arm_velocity)*2;
    wire [63:0] abs_prediction=predicted[63] ? -predicted : predicted;
    wire outward=displacement==0 || displacement[32]==arm_velocity[32];
    wire guard=abs_p>=soft || (outward && abs_prediction>=soft);
    wire brake_positive=arm_velocity[32] ^ (RETURN_POLARITY!=0);
    wire signed [15:0] braking=abs_v<=1 ? 16'sd0 : brake_positive ? brake_cap : -brake_cap;
    wire signed [63:0] return_normal=-$signed(displacement)*16-$signed(arm_velocity)*200;
    wire signed [63:0] return_physical=RETURN_POLARITY!=0 ? -return_normal : return_normal;
    wire signed [15:0] return_command=return_physical>return_cap ? return_cap :
        return_physical < -return_cap ? -return_cap : return_physical[15:0];
    wire [31:0] abs_energy=energy_error_q8[31] ? -energy_error_q8 : energy_error_q8;
    wire energy_quiet=abs_energy<=32;
    wire [15:0] energy_scale=abs_energy>512 ? 16'd512 : abs_energy[15:0];
    wire [31:0] amplitude_product=energy_scale*{1'b0,cap};
    wire [15:0] amplitude=amplitude_product[24:9];
    wire pump_positive=phase_latched ^ energy_error_q8[31] ^ config_words[9*16+2];
    wire pump_normal_positive=pump_positive ^ (RETURN_POLARITY!=0);
    wire pushes_speed=(arm_velocity==0 || pump_normal_positive!=arm_velocity[32]);
    wire [32:0] speed_headroom=abs_v>=speed_cap ? 33'd0 : {17'd0,speed_cap}-abs_v;
    wire [48:0] speed_product=amplitude*speed_headroom;
    wire [15:0] limited_amplitude=pushes_speed && speed_headroom<4 ? speed_product[17:2] : amplitude;
    wire signed [31:0] pump=energy_quiet || phase_age_ms>30 ? 32'sd0 :
        pump_positive ? $signed({1'b0,limited_amplitude}) : -$signed({1'b0,limited_amplitude});
    // Weak drift damping, not endpoint tracking; full recenter only at guard.
    wire signed [63:0] weak_normal=-$signed(displacement)*2-$signed(arm_velocity)*20;
    wire signed [63:0] weak_physical=RETURN_POLARITY!=0 ? -weak_normal : weak_normal;
    wire signed [63:0] weak_bound=weak_physical>return_cap ? return_cap :
        weak_physical < -return_cap ? -return_cap : weak_physical;
    wire signed [63:0] total=$signed(pump)+weak_bound;
    wire signed [15:0] effort=total>cap ? cap : total < -cap ? -cap : total[15:0];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            initialized<=0;seed_active<=0;seed_positive<=0;phase_latched<=0;pending<=0;
            returning<=0;finishing<=0;confirm_count<=0;quiet_ms<=0;phase_age_ms<=0;brake_age_ms<=0;
            command<=0;action<=0;cut_reason<=0;step_ms<=0;last_accel_ms<=0;pulse_index<=0;done<=0;seed_origin<=0;
        end else if(!enable)begin
            initialized<=0;seed_active<=0;returning<=0;finishing<=0;confirm_count<=0;
            quiet_ms<=0;phase_age_ms<=0;brake_age_ms<=0;command<=0;action<=0;done<=0;step_ms<=0;
        end else if(update)begin
            if(step_ms<65000)step_ms<=step_ms+16'd10;
            if(!initialized && (!angle_valid || abs_v>=24 || (capture_zone && config_words[9*16])))begin
                command<=braking;action<=abs_v>=24 ? 4'd6 : angle_valid ? 4'd8 : 4'd9;
            end else if(!initialized)begin
                initialized<=1;seed_active<=1;seed_positive<=(DRIVE_POLARITY==0)^config_words[9*16+2];
                phase_latched<=(DRIVE_POLARITY==0);seed_origin<=displacement;
                pulse_index<=1;step_ms<=0;last_accel_ms<=0;cut_reason<=0;
                command<=((DRIVE_POLARITY==0)^config_words[9*16+2]) ? cap : -cap;action<=15;
            end else if(guard || returning || abs_v>=24)begin
                returning<=1;seed_active<=0;confirm_count<=0;
                command<=outward && abs_v>1 || abs_v>=24 ? braking : return_command;
                action<=abs_v>=24 ? 4'd6 : 4'd7;cut_reason<=abs_v>=24 ? 4'd9 : 4'd4;
                if(outward && abs_v>1)begin
                    if(brake_age_ms+16'd10>=config_words[1*16+:16])begin command<=0;done<=1;cut_reason<=7;end
                    else brake_age_ms<=brake_age_ms+16'd10;
                end else brake_age_ms<=0;
                if(!guard && abs_p<=60 && abs_v<=1)begin returning<=0;step_ms<=0;brake_age_ms<=0;end
            end else if(!angle_valid || (capture_zone && config_words[9*16]))begin
                command<=braking;action<=angle_valid ? 4'd8 : 4'd9;cut_reason<=angle_valid ? 4'd5 : 4'd6;
                seed_active<=0;confirm_count<=0;phase_age_ms<=40;
            end else if(finishing || (!seed_active && pulse_index>=config_words[8*16+:16]))begin
                finishing<=1;seed_active<=0;command<=braking;action<=4;
                if(abs_v<=1)begin command<=0;done<=1;action<=10;end
                else if(brake_age_ms+16'd10>=config_words[1*16+:16])begin command<=0;done<=1;cut_reason<=7;end
                else brake_age_ms<=brake_age_ms+16'd10;
            end else if(seed_active)begin
                if(step_ms+16'd10>=seed_ms || abs_seed_distance>=config_words[5*16+:16] || abs_v>=speed_cap)begin
                    seed_active<=0;last_accel_ms<=step_ms+16'd10;step_ms<=0;
                    command<=0;cut_reason<=10;phase_age_ms<=0;
                end else begin command<=seed_positive ? cap : -cap;action<=15;end
            end else begin
                if(phase_valid)begin
                    phase_age_ms<=0;quiet_ms<=0;
                    if(phase_positive==phase_latched)confirm_count<=0;
                    else if(config_words[10*16+:16]<=1)begin
                        phase_latched<=phase_positive;confirm_count<=0;step_ms<=0;
                        if(!energy_quiet)pulse_index<=pulse_index+1'b1;
                    end
                    else if(confirm_count==0 || pending!=phase_positive)begin pending<=phase_positive;confirm_count<=1;end
                    else if(confirm_count>=config_words[10*16+:16]-1)begin
                        phase_latched<=phase_positive;confirm_count<=0;step_ms<=0;
                        if(!energy_quiet)pulse_index<=pulse_index+1'b1;
                    end else confirm_count<=confirm_count+1'b1;
                end else begin
                    confirm_count<=0;
                    if(phase_age_ms<1000)phase_age_ms<=phase_age_ms+16'd10;
                    if(quiet_ms<1000)quiet_ms<=quiet_ms+16'd10;
                end
                command<=effort;
                action<=energy_quiet ? 4'd13 : pushes_speed && speed_headroom<4 ? 4'd14 :
                    phase_age_ms>30 ? 4'd5 : energy_error_q8[31] ? 4'd13 : 4'd12;
                // A stationary bottom may need another bounded seed, but normal
                // pumping never waits a fixed200ms or stops at a fixed endpoint.
                if(!phase_valid && quiet_ms>=300 && cosine < -180 && abs_v<=1 && energy_error_q8>32)begin
                    seed_positive<=!seed_positive;seed_active<=1;seed_origin<=displacement;step_ms<=0;
                    pulse_index<=pulse_index+1'b1;
                end
            end
        end
    end
endmodule
