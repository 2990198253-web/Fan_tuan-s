// R5 experimental inertia pumping. update is exactly one pulse per 10 ms.
// No endpoint position servo during a kick: bounded acceleration then active
// reverse torque until low speed. This is NOT an electrically shorted brake.
module impulse_swing_controller #(
    parameter integer MAX_COMMAND=4000, KICK_DISTANCE=80,
    parameter integer KICK_SAMPLES=8, KICK_SPEED=10,
    parameter integer BRAKE_SAMPLES=12, SETTLE_SPEED=1,
    parameter integer EMERGENCY_SPEED=16, SOFT_TRAVEL=150,
    parameter integer PREDICT_SAMPLES=8, RETURN_WINDOW=60,
    parameter integer RETURN_COMMAND=1400, PHASE_CONFIRM=2,
    parameter integer COOLDOWN_SAMPLES=8, RESEED_SAMPLES=50,
    parameter integer RETURN_POLARITY=0, DRIVE_POLARITY=0
)(
    input wire clk,rst_n,enable,update,
    input wire signed [32:0] displacement,arm_velocity,
    input wire phase_positive,phase_valid,angle_valid,capture_zone,
    output reg signed [15:0] command,
    output reg [3:0] action
);
    localparam [2:0] START=0,KICK=1,BRAKE=2,WAIT_PHASE=3,RETURN=4;
    localparam signed [15:0] EFFORT=MAX_COMMAND[15:0];
    localparam signed [15:0] RETURN_EFFORT=RETURN_COMMAND[15:0];
    reg [2:0] state;
    reg kick_positive,pending_positive,brake_negative;
    reg [15:0] age,confirm_count;
    reg signed [32:0] kick_origin;
    wire [32:0] abs_position=displacement[32] ? -displacement : displacement;
    wire [32:0] abs_speed=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire signed [33:0] distance=$signed({displacement[32],displacement})-$signed({kick_origin[32],kick_origin});
    wire [33:0] abs_distance=distance[33] ? -distance : distance;
    wire signed [63:0] prediction=$signed(displacement)+$signed(arm_velocity)*PREDICT_SAMPLES;
    wire [63:0] abs_prediction=prediction[63] ? -prediction : prediction;
    wire outward=displacement==0 || displacement[32]==arm_velocity[32];
    wire guard=abs_position>=SOFT_TRAVEL || (outward && abs_prediction>=SOFT_TRAVEL);
    wire brake_positive=arm_velocity[32] ^ (RETURN_POLARITY!=0);
    wire signed [15:0] braking_command=abs_speed<=SETTLE_SPEED ? 16'sd0 :
        brake_positive ? EFFORT : -EFFORT;
    wire signed [63:0] return_effort=-$signed(displacement)*16-$signed(arm_velocity)*200;
    wire signed [63:0] return_physical=RETURN_POLARITY!=0 ? -return_effort : return_effort;
    wire signed [15:0] return_bounded=return_physical>RETURN_COMMAND ? RETURN_EFFORT :
        return_physical < -RETURN_COMMAND ? -RETURN_EFFORT : return_physical[15:0];
    task begin_kick;
        input positive;
        begin
            state<=KICK;kick_positive<=positive;kick_origin<=displacement;
            age<=0;confirm_count<=0;
            command<=positive ? EFFORT : -EFFORT;
            action<=positive ? 4'd2 : 4'd3;
        end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=START;command<=0;action<=0;age<=0;confirm_count<=0;
            kick_origin<=0;kick_positive<=DRIVE_POLARITY==0;
            pending_positive<=0;brake_negative<=0;
        end else if(!enable)begin
            state<=START;command<=0;action<=0;age<=0;confirm_count<=0;
            kick_origin<=0;kick_positive<=DRIVE_POLARITY==0;
            pending_positive<=0;brake_negative<=0;
        end else if(update)begin
            if(age<65535)age<=age+1'b1;
            // Return uses position ONLY to protect travel, never to pump.
            if(state==RETURN || guard)begin
                state<=RETURN;action<=7;confirm_count<=0;
                if(outward && abs_speed>SETTLE_SPEED)command<=braking_command;
                else command<=return_bounded;
                if(!guard && abs_position<=RETURN_WINDOW && abs_speed<=SETTLE_SPEED)begin
                    state<=WAIT_PHASE;age<=0;command<=0;action<=5;
                end
            end else if(capture_zone || !angle_valid)begin
                // Rail/dead-zone samples must not request a new full kick.
                state<=WAIT_PHASE;age<=0;confirm_count<=0;
                command<=braking_command;action<=capture_zone ? 4'd8 : 4'd9;
            end else if(abs_speed>=EMERGENCY_SPEED)begin
                state<=BRAKE;age<=0;command<=braking_command;action<=6;
                brake_negative<=arm_velocity[32];confirm_count<=0;
            end else case(state)
                START:begin_kick(DRIVE_POLARITY==0);
                KICK:begin
                    if(age>=KICK_SAMPLES-1 || abs_distance>=KICK_DISTANCE || abs_speed>=KICK_SPEED)begin
                        state<=BRAKE;age<=0;command<=braking_command;action<=4;
                        brake_negative<=arm_velocity[32];
                    end else begin command<=kick_positive ? EFFORT : -EFFORT;
                        action<=kick_positive ? 4'd2 : 4'd3;end
                end
                BRAKE:begin
                    action<=4;command<=braking_command;
                    if(abs_speed<=SETTLE_SPEED || arm_velocity[32]!=brake_negative)begin
                        state<=WAIT_PHASE;age<=0;command<=0;action<=5;
                    end else if(age>=BRAKE_SAMPLES-1)begin
                        // Do not turn a failed deceleration into unlimited reverse drive.
                        state<=RETURN;age<=0;command<=braking_command;action<=7;
                    end
                end
                default:begin
                    command<=0;action<=5;
                    if(age>=COOLDOWN_SAMPLES && phase_valid && phase_positive!=kick_positive)begin
                        if(confirm_count==0 || pending_positive!=phase_positive)begin
                            pending_positive<=phase_positive;confirm_count<=1;
                        end else if(confirm_count>=PHASE_CONFIRM-1)begin_kick(phase_positive);
                        else confirm_count<=confirm_count+1'b1;
                    end else begin
                        confirm_count<=0;
                        // Restart a motionless bottom, not a fixed-frequency pump.
                        if(age>=RESEED_SAMPLES && !phase_valid && abs_speed<=SETTLE_SPEED)
                            begin_kick(!kick_positive);
                    end
                end
            endcase
        end
    end
endmodule
