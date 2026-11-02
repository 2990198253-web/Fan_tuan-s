// Jiangxie-style cascade: position PID changes the angle target, never PWM.
// Outer e = target - position; angle target offset = -outer output.
// Q8 gains/output allow fractional ADC targets without integer dead zones.
module position_pid_controller #(
    parameter signed [15:0] KP_Q8 = 16'sd2,
    parameter signed [15:0] KD_Q8 = 16'sd64,
    parameter signed [15:0] KI_Q16 = 16'sd0,
    parameter integer I_LIMIT_ADC = 2,
    parameter integer INTEGRAL_WINDOW_COUNTS = 0,
    parameter integer MAX_OFFSET_ADC = 6,
    parameter integer MAX_OFFSET_STEP_Q8 = 64,
    // Soft position wall around the current trajectory target. Requested
    // positions retain their original range; motor power is never cut here.
    parameter integer SOFT_BOUND_COUNTS = 0,
    parameter integer SOFT_BOUND_MARGIN_COUNTS = 4,
    parameter integer SOFT_BOUND_GAIN_Q8 = 128,
    // On the lab top, all-zero live outer gains mean a true inner-only trial.
    parameter integer DISABLE_GUARD_WITH_ZERO_GAINS = 0,
    // Compress large position corrections before the hard angle limit.
    // Zero preserves the original direct clamp for other top-level users.
    parameter integer SOFT_CLIP_KNEE_ADC = 0,
    parameter integer VELOCITY_FILTER_SHIFT = 1,
    parameter integer TARGET_MODE = 0,
    parameter signed [31:0] TARGET_COUNTS = 32'sd0,
    parameter integer OUTPUT_POLARITY = 0,
    parameter integer RUNTIME_GAINS = 0
) (
    input wire clk, rst_n, enable, capture, update,
    input wire [15:0] runtime_kp_q8,runtime_ki_q16,runtime_kd_q8,
    input wire reset_integral,
    input wire move_request,
    input wire signed [15:0] move_relative,
    input wire [15:0] move_speed,
    input wire signed [31:0] position,
    output reg signed [31:0] target_offset_q8,
    output reg signed [31:0] target_position,
    output reg signed [31:0] anchor_position,
    output wire signed [32:0] position_error,
    output reg signed [31:0] velocity_q8
);
    reg signed [31:0] previous_position, integral_q16, goal_position;
    reg [5:0] requested_speed, profile_speed;
    reg [1:0] accel_phase;
    reg [7:0] travel_credit;
    reg signed [32:0] error_hold;
    reg signed [31:0] integral_candidate_q16, limited_hold;
    reg signed [48:0] p_hold, d_hold;
    reg signed [48:0] guard_hold;
    reg signed [47:0] d_product_hold;
    reg signed [48:0] i_product_hold;
    reg signed [15:0] ki_sample_hold;
    reg signed [49:0] outer_sum_hold;
    reg [2:0] stage;
    assign position_error = $signed({target_position[31],target_position}) -
                            $signed({position[31],position});
    wire signed [32:0] delta = $signed({position[31],position}) -
                              $signed({previous_position[31],previous_position});
    // Clamp an impossible per-update delta before Q8 conversion, not a stop.
    wire signed [31:0] bounded_delta = (delta > 32767) ? 32'sd32767 :
        (delta < -32767) ? -32'sd32767 : {{16{delta[15]}},delta[15:0]};
    wire signed [31:0] step_q8 = bounded_delta <<< 8;
    wire signed [31:0] velocity_filtered = velocity_q8 +
        ((step_q8 - velocity_q8) >>> VELOCITY_FILTER_SHIFT);
    wire signed [31:0] velocity_next = (step_q8 == 0 &&
        velocity_q8 < (1 << VELOCITY_FILTER_SHIFT) && velocity_q8 > -(1 << VELOCITY_FILTER_SHIFT)) ?
        32'sd0 : velocity_filtered;
    // The live protocol accepts P=0..32 and D=0..8191. Trim the multiplier
    // inputs to the required widths; a full 16-bit variable D misses 50 MHz.
    wire signed [15:0] active_kp_q8 = RUNTIME_GAINS ? $signed({10'd0,runtime_kp_q8[5:0]}) : KP_Q8;
    wire signed [15:0] active_kd_q8 = RUNTIME_GAINS ? $signed({3'd0,runtime_kd_q8[12:0]}) : KD_Q8;
    wire signed [15:0] active_ki_q16 = RUNTIME_GAINS ? $signed({9'd0,runtime_ki_q16[6:0]}) : KI_Q16;
    wire signed [48:0] p_term = error_hold * active_kp_q8;
    localparam integer GUARD_START_COUNTS = SOFT_BOUND_COUNTS > SOFT_BOUND_MARGIN_COUNTS ?
        SOFT_BOUND_COUNTS - SOFT_BOUND_MARGIN_COUNTS : 0;
    wire signed [32:0] guard_error = SOFT_BOUND_COUNTS <= 0 ? 33'sd0 :
        (error_hold > GUARD_START_COUNTS) ?
            error_hold - GUARD_START_COUNTS :
        (error_hold < -GUARD_START_COUNTS) ?
            error_hold + GUARD_START_COUNTS : 33'sd0;
    wire outer_gains_zero = active_kp_q8 == 0 && active_ki_q16 == 0 && active_kd_q8 == 0;
    wire signed [32:0] active_guard_error =
        (DISABLE_GUARD_WITH_ZERO_GAINS != 0 && outer_gains_zero) ? 33'sd0 : guard_error;
    wire signed [48:0] guard_term = active_guard_error * SOFT_BOUND_GAIN_Q8;
    wire signed [47:0] d_product = velocity_q8 * active_kd_q8;
    wire signed [48:0] d_term = -(d_product_hold[47] ? -((-d_product_hold) >>> 8) : (d_product_hold >>> 8));
    wire signed [48:0] i_product = error_hold * active_ki_q16;
    // Retain the full Q16 increment: small count errors with KI=1 must not
    // disappear in a per-update Q8 right shift.
    wire signed [49:0] i_sum = $signed(integral_q16) + i_product_hold;
    wire integral_allowed = INTEGRAL_WINDOW_COUNTS <= 0 ||
        (error_hold <= INTEGRAL_WINDOW_COUNTS && error_hold >= -INTEGRAL_WINDOW_COUNTS);
    wire signed [31:0] i_next = (ki_sample_hold == 0 || !integral_allowed) ? 32'sd0 :
        (i_sum > I_LIMIT_ADC * 65536) ? I_LIMIT_ADC * 65536 :
        (i_sum < -I_LIMIT_ADC * 65536) ? -I_LIMIT_ADC * 65536 : i_sum[31:0];
    wire signed [31:0] i_output_q8 = integral_candidate_q16[31] ?
        -((-$signed(integral_candidate_q16)) >>> 8) : (integral_candidate_q16 >>> 8);
    wire signed [49:0] outer_sum = $signed(p_hold) + $signed(d_hold) + $signed(i_output_q8);
    wire signed [49:0] offset_request = OUTPUT_POLARITY ? outer_sum_hold : -outer_sum_hold;
    localparam integer CLIP_KNEE_Q8 = SOFT_CLIP_KNEE_ADC * 256;
    localparam integer CLIP_INPUT_LIMIT_Q8 = (2 * MAX_OFFSET_ADC - SOFT_CLIP_KNEE_ADC) * 256;
    wire signed [31:0] bounded_request =
        (offset_request > CLIP_INPUT_LIMIT_Q8) ? CLIP_INPUT_LIMIT_Q8 :
        (offset_request < -CLIP_INPUT_LIMIT_Q8) ? -CLIP_INPUT_LIMIT_Q8 : offset_request[31:0];
    wire signed [31:0] soft_offset =
        (bounded_request > CLIP_KNEE_Q8) ?
            CLIP_KNEE_Q8 + ((bounded_request - CLIP_KNEE_Q8) >>> 1) :
        (bounded_request < -CLIP_KNEE_Q8) ?
            -CLIP_KNEE_Q8 - (((-bounded_request) - CLIP_KNEE_Q8) >>> 1) : bounded_request;
    wire signed [31:0] limited_offset =
        (SOFT_CLIP_KNEE_ADC > 0 && SOFT_CLIP_KNEE_ADC < MAX_OFFSET_ADC) ? soft_offset :
        (offset_request > MAX_OFFSET_ADC * 256) ? MAX_OFFSET_ADC * 256 :
        (offset_request < -MAX_OFFSET_ADC * 256) ? -MAX_OFFSET_ADC * 256 : offset_request[31:0];
    wire signed [32:0] offset_step = $signed({limited_hold[31],limited_hold}) -
                                   $signed({target_offset_q8[31],target_offset_q8});
    // The outer target follows a bounded trapezoidal velocity profile.
    // One update is 5 ms: 200 updates/s, at most 30 encoder counts/s.
    wire signed [32:0] remaining = $signed({goal_position[31],goal_position}) -
                                   $signed({target_position[31],target_position});
    wire signed [31:0] requested_goal = anchor_position + {{16{move_relative[15]}},move_relative};
    wire [32:0] distance = remaining[32] ? -remaining : remaining;
    wire [11:0] speed_squared = profile_speed * profile_speed;
    wire [8:0] brake_distance = ((speed_squared + 12'd127) >> 7) + 9'd1;
    wire [8:0] credit_sum = {1'b0,travel_credit} + {3'd0,profile_speed};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_offset_q8 <= 0; target_position <= 0; anchor_position <= 0;
            goal_position <= 0; requested_speed <= 0; profile_speed <= 0;
            accel_phase <= 0; travel_credit <= 0; previous_position <= 0;
            velocity_q8 <= 0; integral_q16 <= 0; integral_candidate_q16 <= 0;
            error_hold <= 0; p_hold <= 0; d_hold <= 0; guard_hold <= 0;
            d_product_hold <= 0; i_product_hold <= 0; ki_sample_hold <= 0;
            outer_sum_hold <= 0; limited_hold <= 0; stage <= 0;
        end else if (capture) begin
            target_position <= (TARGET_MODE == 0) ? position + TARGET_COUNTS : TARGET_COUNTS;
            goal_position <= (TARGET_MODE == 0) ? position + TARGET_COUNTS : TARGET_COUNTS;
            anchor_position <= position;
            requested_speed <= 0; profile_speed <= 0; accel_phase <= 0; travel_credit <= 0;
            previous_position <= position; velocity_q8 <= 0; integral_q16 <= 0;
            integral_candidate_q16 <= 0; target_offset_q8 <= 0; stage <= 0;
        end else if (reset_integral) begin
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            i_product_hold <= 0; ki_sample_hold <= 0; stage <= 0;
        end else if (move_request && enable) begin
            goal_position <= requested_goal;
            requested_speed <= move_speed[5:0];
            // Repeated UI commands for the same goal must not restart the
            // motion profile. Keep speed/credit when extending a move in its
            // current direction; restart only for a reversal or a stop.
            if (requested_goal == target_position ||
                (requested_goal > target_position && goal_position < target_position) ||
                (requested_goal < target_position && goal_position > target_position)) begin
                profile_speed <= 0; accel_phase <= 0; travel_credit <= 0;
            end
        end else if (!enable) begin
            previous_position <= position; velocity_q8 <= 0; integral_q16 <= 0;
            integral_candidate_q16 <= 0; target_offset_q8 <= 0; stage <= 0;
            profile_speed <= 0; accel_phase <= 0; travel_credit <= 0;
        end else if (update) begin
            previous_position <= position; velocity_q8 <= velocity_next;
            error_hold <= position_error; stage <= 1;
            if (remaining == 0) begin
                profile_speed <= 0; accel_phase <= 0; travel_credit <= 0;
            end else begin
                if (accel_phase == 3) begin
                    accel_phase <= 0;
                    if (profile_speed == 0) profile_speed <= 1;
                    else if (distance <= brake_distance && profile_speed > 1)
                        profile_speed <= profile_speed - 1'b1;
                    else if (distance > brake_distance && profile_speed < requested_speed)
                        profile_speed <= profile_speed + 1'b1;
                end else accel_phase <= accel_phase + 1'b1;
                if (credit_sum >= 9'd200) begin
                    target_position <= target_position + (remaining[32] ? -32'sd1 : 32'sd1);
                    travel_credit <= credit_sum - 9'd200;
                end else travel_credit <= credit_sum[7:0];
            end
        end else begin
            case (stage)
                1: begin
                    p_hold <= p_term; guard_hold <= guard_term;
                    d_product_hold <= d_product;
                    i_product_hold <= i_product; ki_sample_hold <= active_ki_q16; stage <= 2;
                end
                2: begin d_hold <= d_term; p_hold <= p_hold + guard_hold;
                    integral_candidate_q16 <= i_next; stage <= 3; end
                3: begin outer_sum_hold <= outer_sum; stage <= 4; end
                4: begin
                    limited_hold <= limited_offset;
                    if (ki_sample_hold == 0 || !integral_allowed) integral_q16 <= 0;
                    else if (!((outer_sum_hold > MAX_OFFSET_ADC * 256 && error_hold > 0) ||
                               (outer_sum_hold < -MAX_OFFSET_ADC * 256 && error_hold < 0)))
                        integral_q16 <= integral_candidate_q16;
                    stage <= 5;
                end
                5: begin
                    if (MAX_OFFSET_STEP_Q8 > 0 && offset_step > MAX_OFFSET_STEP_Q8)
                        target_offset_q8 <= target_offset_q8 + MAX_OFFSET_STEP_Q8;
                    else if (MAX_OFFSET_STEP_Q8 > 0 && offset_step < -MAX_OFFSET_STEP_Q8)
                        target_offset_q8 <= target_offset_q8 - MAX_OFFSET_STEP_Q8;
                    else target_offset_q8 <= limited_hold;
                    stage <= 0;
                end
                default: stage <= 0;
            endcase
        end
    end
endmodule
