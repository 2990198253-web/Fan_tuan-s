// 100 Hz experiment sequencer. The requested acceleration duration is used
// unless an explicitly reported safety condition interrupts it.
module swing_lab_controller #(
    parameter integer RETURN_POLARITY=0,DRIVE_POLARITY=0
)(
    input wire clk,rst_n,enable,update,
    input wire [223:0] config_words,
    input wire signed [32:0] displacement,arm_velocity,
    input wire phase_positive,phase_valid,angle_valid,capture_zone,
    output reg signed [15:0] command,
    output reg [3:0] action,cut_reason,
    output reg [15:0] step_ms,last_accel_ms,pulse_index,
    output reg done
);
    localparam [2:0] INIT=0,ACCEL=1,BRAKE=2,WAITING=3,RETURNING=4,FINISHED=5;
    reg [2:0] state;reg direction,brake_negative,pending;
    reg [15:0] confirm_count;reg signed [32:0] kick_origin;
    wire [15:0] accel_ms=config_words[0*16+:16],brake_ms=config_words[1*16+:16];
    wire [15:0] wait_ms=config_words[2*16+:16],distance_limit=config_words[5*16+:16];
    wire signed [15:0] accel_pwm=$signed(config_words[3*16+:16]),brake_pwm=$signed(config_words[4*16+:16]);
    wire [15:0] target_speed=config_words[6*16+:16],soft_limit=config_words[7*16+:16];
    wire [15:0] pulses=config_words[8*16+:16],flags=config_words[9*16+:16];
    wire signed [15:0] return_pwm=$signed(config_words[12*16+:16]);
    wire [32:0] abs_position=displacement[32] ? -displacement : displacement;
    wire [32:0] abs_speed=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire signed [33:0] distance=$signed({displacement[32],displacement})-$signed({kick_origin[32],kick_origin});
    wire [33:0] abs_distance=distance[33] ? -distance : distance;
    wire outward=displacement==0 || displacement[32]==arm_velocity[32];
    // Only 20 ms look-ahead, not the R5 80 ms early return prediction.
    wire signed [63:0] predicted=$signed(displacement)+$signed(arm_velocity)*2;
    wire [63:0] abs_predicted=predicted[63] ? -predicted : predicted;
    wire guard=abs_position>=soft_limit || (outward && abs_predicted>=soft_limit);
    wire brake_positive=arm_velocity[32] ^ (RETURN_POLARITY!=0);
    wire signed [15:0] brake_command=abs_speed<=1 ? 16'sd0 : brake_positive ? brake_pwm : -brake_pwm;
    wire signed [63:0] re=-$signed(displacement)*16-$signed(arm_velocity)*200;
    wire signed [63:0] rp=RETURN_POLARITY!=0 ? -re : re;
    wire signed [15:0] return_command=rp>return_pwm ? return_pwm : rp < -return_pwm ? -return_pwm : rp[15:0];
    wire target_positive=direction ^ (RETURN_POLARITY!=0);
    wire signed [33:0] speed_request=target_positive ? $signed({1'b0,target_speed}) : -$signed({1'b0,target_speed});
    wire signed [63:0] speed_error=$signed(speed_request)-$signed(arm_velocity);
    wire signed [63:0] normalized_speed_effort=speed_error*200+$signed(speed_request)*70;
    wire signed [63:0] speed_effort=RETURN_POLARITY!=0 ? -normalized_speed_effort : normalized_speed_effort;
    wire signed [15:0] speed_command=speed_effort>accel_pwm ? accel_pwm : speed_effort < -accel_pwm ? -accel_pwm : speed_effort[15:0];
    wire at_speed=speed_error>=-1 && speed_error<=1;
    function signed [15:0] first_speed_command;
        input positive;
        reg signed [63:0] requested,error,effort,physical;
        begin
            requested=(positive ^ (RETURN_POLARITY!=0)) ? $signed({1'b0,target_speed}) : -$signed({1'b0,target_speed});
            error=requested-$signed(arm_velocity);effort=error*200+requested*70;
            physical=RETURN_POLARITY!=0 ? -effort : effort;
            first_speed_command=physical>accel_pwm ? accel_pwm : physical < -accel_pwm ? -accel_pwm : physical[15:0];
        end
    endfunction
    task kick;
        input positive;
        begin state<=ACCEL;direction<=positive;kick_origin<=displacement;
            step_ms<=0;pulse_index<=pulse_index+1'b1;confirm_count<=0;
            command<=flags[3] ? first_speed_command(positive) : positive ? accel_pwm : -accel_pwm;action<=positive ? 4'd2 : 4'd3;
        end
    endtask
    task brake;
        input [3:0] reason;
        begin state<=BRAKE;cut_reason<=reason;last_accel_ms<=step_ms+16'd10;
            step_ms<=0;brake_negative<=arm_velocity[32];command<=brake_command;action<=4;
        end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=INIT;command<=0;action<=0;cut_reason<=0;step_ms<=0;last_accel_ms<=0;
            pulse_index<=0;done<=0;direction<=0;brake_negative<=0;pending<=0;confirm_count<=0;kick_origin<=0;
        end else if(!enable)begin
            // Keep completed trial measurements available in IDLE/FAULT.
            if(state==ACCEL)begin last_accel_ms<=step_ms;cut_reason<=8;end
            state<=INIT;command<=0;action<=0;done<=0;confirm_count<=0;step_ms<=0;
            kick_origin<=0;
        end else if(update)begin
            if(step_ms<65000)step_ms<=step_ms+16'd10;
            if(state==FINISHED)begin command<=0;done<=1;action<=10;end
            else if(abs_speed>=24)begin
                // Fixed, non-configurable ultimate speed safety.
                state<=RETURNING;command<=brake_command;action<=6;cut_reason<=9;
                if(state==ACCEL)last_accel_ms<=step_ms+16'd10;
                step_ms<=0;
            end else if(guard && state!=RETURNING)begin
                state<=RETURNING;command<=brake_command;action<=7;cut_reason<=4;step_ms<=0;
                if(state==ACCEL)last_accel_ms<=step_ms+16'd10;
            end else if(state!=RETURNING && (!angle_valid || (capture_zone && flags[0])))begin
                state<=BRAKE;command<=brake_command;action<=angle_valid ? 4'd8 : 4'd9;
                cut_reason<=angle_valid ? 4'd5 : 4'd6;brake_negative<=arm_velocity[32];step_ms<=0;
                if(state==ACCEL)last_accel_ms<=step_ms+16'd10;
            end else case(state)
                INIT:begin last_accel_ms<=0;cut_reason<=0;kick((DRIVE_POLARITY==0)^flags[2]);pulse_index<=1;end
                ACCEL:begin
                    if(abs_distance>=distance_limit)brake(2);
                    else if(step_ms+16'd10>=accel_ms)brake(1);
                    else begin
                        command<=flags[3] ? speed_command : direction ? accel_pwm : -accel_pwm;
                        action<=flags[3] && at_speed ? 4'd11 : direction ? 4'd2 : 4'd3;
                    end
                end
                BRAKE:begin
                    command<=brake_command;action<=4;
                    if(abs_speed<=1 || arm_velocity[32]!=brake_negative)begin
                        state<=WAITING;step_ms<=0;command<=0;action<=5;
                    end else if(step_ms+16'd10>=brake_ms)begin
                        // A failed brake is a failed trial, NOT an unbounded reverse drive.
                        state<=FINISHED;command<=0;done<=1;cut_reason<=7;action<=10;
                    end
                end
                RETURNING:begin
                    action<=7;
                    command<=outward && abs_speed>1 ? brake_command : return_command;
                    if(!guard && abs_position<=60 && abs_speed<=1)begin
                        state<=WAITING;step_ms<=0;command<=0;action<=5;
                    end
                end
                default:begin
                    command<=0;action<=5;
                    if(pulse_index>=pulses)begin state<=FINISHED;done<=1;action<=10;end
                    else if(step_ms+16'd10>=wait_ms)begin
                        if(!flags[1])kick(!direction);
                        else if(phase_valid && phase_positive!=direction)begin
                            if(confirm_count==0 || pending!=phase_positive)begin pending<=phase_positive;confirm_count<=1;end
                            else if(confirm_count>=config_words[10*16+:16]-1)kick(phase_positive);
                            else confirm_count<=confirm_count+1'b1;
                        end else confirm_count<=0;
                    end
                end
            endcase
        end
    end
endmodule
