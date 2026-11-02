// 100 Hz bounded phase-synchronous arm position/velocity cascade.
// Signed positions are relative to the K1 origin; velocity is counts/10ms.
// Motor/encoder direction is independent of the upright position-loop sign.
module swing_trajectory_controller #(
    parameter integer START_STROKE=60, MAX_STROKE=120, STROKE_STEP=10,
    parameter integer PHASE_CONFIRM=2, PHASE_HOLD=8,
    parameter integer MAX_SPEED=8, EMERGENCY_SPEED=12,
    parameter integer POSITION_GAIN_Q8=24,
    parameter integer VELOCITY_GAIN=200, FEEDFORWARD_GAIN=70,
    parameter integer ACCEL_STEP_Q8=128,
    parameter integer MAX_COMMAND=1400,
    parameter integer SOFT_TRAVEL=150, PREDICT_SAMPLES=8,
    parameter integer RETURN_WINDOW=60, SETTLE_SPEED=2,
    parameter integer RETURN_POLARITY=0, DRIVE_POLARITY=0
)(
    input wire clk,rst_n,enable,update,
    input wire signed [32:0] displacement, arm_velocity,
    input wire phase_positive, phase_valid, energy_quiet, energy_low,
    input wire capture_zone,
    input wire [15:0] effort_limit,
    output reg signed [15:0] command,
    output wire signed [7:0] target_offset,
    output reg [3:0] action,
    output reg [7:0] stroke
);
    reg initialized, direction_positive, pending_direction, returning;
    reg half_cycle;
    reg [15:0] confirm_count,hold_count;
    reg signed [31:0] velocity_reference_q8;
    localparam integer STROKE_MARGIN=SOFT_TRAVEL>20 ? 20 : SOFT_TRAVEL/4;
    localparam integer REQUESTED_LIMIT=MAX_STROKE<127 ? MAX_STROKE : 127;
    localparam integer STROKE_LIMIT=REQUESTED_LIMIT<SOFT_TRAVEL-STROKE_MARGIN ?
        REQUESTED_LIMIT : SOFT_TRAVEL-STROKE_MARGIN;
    localparam integer START_LEVEL=START_STROKE<STROKE_LIMIT ? START_STROKE : STROKE_LIMIT;
    wire [32:0] abs_position=displacement[32] ? -displacement : displacement;
    wire [32:0] abs_velocity=arm_velocity[32] ? -arm_velocity : arm_velocity;
    wire outward=displacement==0 || displacement[32]==arm_velocity[32];
    wire signed [63:0] prediction=$signed(displacement)+$signed(arm_velocity)*PREDICT_SAMPLES;
    wire [63:0] abs_prediction=prediction[63] ? -prediction : prediction;
    wire guard_needed=abs_position>=SOFT_TRAVEL ||
        (PREDICT_SAMPLES>0 && outward && abs_prediction>=SOFT_TRAVEL);
    wire guard_active=guard_needed || returning;
    wire target_positive=direction_positive ^ (RETURN_POLARITY!=0);
    wire signed [31:0] track_target=target_positive ? $signed({1'b0,stroke}) : -$signed({1'b0,stroke});
    wire signed [31:0] desired_target=guard_active || capture_zone || energy_quiet ? 0 : track_target;
    // Capture/energy waiting damps in place, not a last-minute move to origin.
    wire signed [63:0] position_error=(capture_zone || energy_quiet) && !guard_active ?
        64'sd0 : $signed(desired_target)-$signed(displacement);
    wire signed [63:0] velocity_from_position=position_error*POSITION_GAIN_Q8;
    wire signed [31:0] speed_target_q8=velocity_from_position>MAX_SPEED*256 ? MAX_SPEED*256 :
        velocity_from_position < -MAX_SPEED*256 ? -MAX_SPEED*256 : velocity_from_position[31:0];
    wire signed [31:0] desired_velocity_q8=guard_active ? speed_target_q8 :
        capture_zone || energy_quiet ? 32'sd0 : speed_target_q8;
    wire signed [31:0] next_velocity_q8=desired_velocity_q8>velocity_reference_q8+ACCEL_STEP_Q8 ?
        velocity_reference_q8+ACCEL_STEP_Q8 : desired_velocity_q8<velocity_reference_q8-ACCEL_STEP_Q8 ?
        velocity_reference_q8-ACCEL_STEP_Q8 : desired_velocity_q8;
    wire signed [63:0] velocity_error_q8=$signed(next_velocity_q8)-($signed(arm_velocity)*256);
    wire signed [63:0] normalized_effort=(velocity_error_q8*VELOCITY_GAIN+
        $signed(next_velocity_q8)*FEEDFORWARD_GAIN) >>> 8;
    wire signed [63:0] physical_effort=RETURN_POLARITY!=0 ? -normalized_effort : normalized_effort;
    wire [15:0] cap=effort_limit>MAX_COMMAND ? MAX_COMMAND[15:0] : effort_limit;
    wire [31:0] grown_stroke={24'd0,stroke}+STROKE_STEP;
    wire signed [15:0] bounded_effort=physical_effort>$signed({1'b0,cap}) ? $signed(cap) :
        physical_effort < -$signed({1'b0,cap}) ? -$signed(cap) : physical_effort[15:0];
    wire brake_positive=arm_velocity[32] ? (RETURN_POLARITY==0) : (RETURN_POLARITY!=0);
    assign target_offset=guard_active || capture_zone || energy_quiet ? 8'sd0 : track_target[7:0];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            initialized<=0;direction_positive<=(DRIVE_POLARITY==0);pending_direction<=0;
            returning<=0;half_cycle<=0;confirm_count<=0;hold_count<=0;
            velocity_reference_q8<=0;command<=0;action<=0;stroke<=START_LEVEL[7:0];
        end else if(!enable)begin
            initialized<=0;direction_positive<=(DRIVE_POLARITY==0);pending_direction<=0;
            returning<=0;half_cycle<=0;confirm_count<=0;hold_count<=0;
            velocity_reference_q8<=0;command<=0;action<=0;stroke<=START_LEVEL[7:0];
        end else if(update)begin
            initialized<=1;
            if(hold_count<PHASE_HOLD)hold_count<=hold_count+1'b1;
            if(guard_needed)returning<=1;
            else if(returning && abs_position<=RETURN_WINDOW && abs_velocity<=SETTLE_SPEED)returning<=0;
            // A phase request must persist and the previous phase must last
            // long enough. No time-only toggling when the pole is stationary.
            if(!phase_valid || phase_positive==direction_positive || capture_zone || guard_active || energy_quiet)confirm_count<=0;
            else if(hold_count>=PHASE_HOLD)begin
                if(confirm_count==0 || pending_direction!=phase_positive)begin
                    pending_direction<=phase_positive;confirm_count<=1;
                end else if(confirm_count>=PHASE_CONFIRM-1)begin
                    direction_positive<=phase_positive;confirm_count<=0;hold_count<=0;
                    half_cycle<=!half_cycle;
                    if(half_cycle && energy_low && stroke<STROKE_LIMIT)
                        stroke<=grown_stroke>STROKE_LIMIT ? STROKE_LIMIT[7:0] : grown_stroke[7:0];
                end else confirm_count<=confirm_count+1'b1;
            end
            velocity_reference_q8<=next_velocity_q8;
            // Large speed anomaly remains a full bounded reverse action. The
            // ordinary speed target is handled by continuous feedback, NOT
            // the old speed5-to-speed2 relay/full reverse override.
            if(EMERGENCY_SPEED>0 && abs_velocity>=EMERGENCY_SPEED)begin
                command<=brake_positive ? $signed(cap) : -$signed(cap);action<=6;
                velocity_reference_q8<=0;
            end else begin
                command<=bounded_effort;
                if(guard_active)action<=7;
                else if(capture_zone)action<=8;
                else if(energy_quiet)action<=9;
                else if(!initialized)action<=1;
                else if(position_error<=3 && position_error>=-3 && abs_velocity<=SETTLE_SPEED)action<=5;
                else if((arm_velocity>0 && normalized_effort<0) || (arm_velocity<0 && normalized_effort>0))action<=4;
                else action<=target_positive ? 4'd2 : 4'd3;
            end
        end
    end
endmodule
