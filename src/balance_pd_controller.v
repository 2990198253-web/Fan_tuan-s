// Half-count PID, five-sample center qualification, measured-velocity D.
// Position target always remains effective, even when angle P is suppressed.
module balance_pd_controller #(
    parameter integer UPRIGHT_ADC_X2 = 1527,
    parameter integer LIVE_UPRIGHT_ENABLE=0,
    parameter integer ANGLE_DEADBAND_X2 = 1,
    parameter integer DEADBAND_CONFIRM_TICKS = 5,
    // Legacy mode suppresses angle P after center qualification. Smooth mode
    // retains restoring P, avoiding a second discontinuity near upright.
    parameter integer CENTER_P_SUPPRESS = 1,
    parameter integer CONTROL_DIV = 25000,
    parameter signed [15:0] KP = 16'sd250,
    parameter signed [15:0] KD = 16'sd100,
    parameter signed [15:0] KI_Q8 = 16'sd0,
    parameter integer RUNTIME_GAINS = 0,
    parameter integer INTEGRAL_LIMIT_COMMAND = 300,
    parameter integer INTEGRAL_WINDOW_ADC = 10,
    parameter integer MAX_D_COMMAND = 1000,
    parameter integer MAX_COMMAND = 2000,
    parameter integer DERIVATIVE_WINDOW_TICKS = 4,
    parameter integer DERIVATIVE_FILTER_SHIFT = 1,
    parameter integer MOTOR_POLARITY = 0
) (
    input wire clk, rst_n, enable, update, reset_integral,
    input wire [15:0] runtime_kp,runtime_ki_q8,runtime_kd,
    input wire [10:0] angle_x2,
    input wire [10:0] upright_adc_x2,
    input wire signed [31:0] target_offset_q8,
    output reg signed [15:0] command,
    output reg command_update,
    output reg inside_deadband, deadband_confirmed
);
    localparam integer N = DERIVATIVE_WINDOW_TICKS > 0 ? DERIVATIVE_WINDOW_TICKS : 1;
    // Fixed board-calibrated center. Removed the unused reference-dither mode.
    wire signed [31:0] live_upright_adc_x2 = LIVE_UPRIGHT_ENABLE!=0 ? $signed({1'b0,upright_adc_x2}) : UPRIGHT_ADC_X2;
    // delta is in half-counts; D stays in whole ADC counts per 5ms reference.
    localparam integer VELOCITY_SCALE_Q8 = (250000 * 256) / (CONTROL_DIV * N * 2);
    reg signed [11:0] history [0:N-1];
    reg history_ready;
    reg [31:0] history_index, center_count;
    reg signed [31:0] velocity_q8, error_hold_q8;
    reg signed [47:0] p_hold, d_hold;
    // Q16 preserves half-count/sub-count integration instead of discarding
    // every fractional increment in a Q8 right shift.
    reg signed [31:0] integral_q16, integral_candidate_q16;
    reg signed [48:0] sum_hold,raw_hold;
    reg [2:0] stage;
    integer i;
    wire signed [11:0] measurement = $signed({1'b0,angle_x2});
    wire signed [31:0] measured_error_x2 = measurement - live_upright_adc_x2;
    wire center_now = measured_error_x2 <= ANGLE_DEADBAND_X2 && measured_error_x2 >= -ANGLE_DEADBAND_X2;
    wire qualified_now = center_now && (DEADBAND_CONFIRM_TICKS <= 1 || center_count >= DEADBAND_CONFIRM_TICKS - 1);
    wire signed [31:0] angle_p_error_q8 = (CENTER_P_SUPPRESS != 0 && qualified_now) ?
        32'sd0 : (measured_error_x2 <<< 7);
    wire signed [31:0] effective_error_q8 = angle_p_error_q8 - target_offset_q8;
    wire signed [11:0] delta_x2 = measurement - history[history_index];
    wire signed [31:0] measured_velocity = delta_x2 * VELOCITY_SCALE_Q8;
    wire signed [31:0] filtered_velocity = velocity_q8 + ((measured_velocity - velocity_q8) >>> DERIVATIVE_FILTER_SHIFT);
    wire signed [31:0] velocity_next = (measured_velocity == 0 && velocity_q8 < (1 << DERIVATIVE_FILTER_SHIFT) &&
        velocity_q8 > -(1 << DERIVATIVE_FILTER_SHIFT)) ? 32'sd0 : filtered_velocity;
    wire signed [15:0] active_kp = RUNTIME_GAINS ? $signed(runtime_kp) : KP;
    wire signed [15:0] active_kd = RUNTIME_GAINS ? $signed(runtime_kd) : KD;
    wire signed [15:0] active_ki_q8 = RUNTIME_GAINS ? $signed(runtime_ki_q8) : KI_Q8;
    wire signed [47:0] p_product = error_hold_q8 * active_kp;
    wire signed [47:0] d_product = velocity_q8 * active_kd;
    wire signed [47:0] d_limited = (d_product > MAX_D_COMMAND * 256) ? MAX_D_COMMAND * 256 :
        (d_product < -MAX_D_COMMAND * 256) ? -MAX_D_COMMAND * 256 : d_product;
    wire integral_allowed = INTEGRAL_WINDOW_ADC <= 0 ||
        (error_hold_q8 <= INTEGRAL_WINDOW_ADC * 256 && error_hold_q8 >= -INTEGRAL_WINDOW_ADC * 256);
    wire signed [47:0] integral_step_q16 = error_hold_q8 * active_ki_q8;
    wire signed [48:0] integral_sum_q16 = $signed(integral_q16) + integral_step_q16;
    wire signed [31:0] integral_next_q16 = (active_ki_q8 == 0 || !integral_allowed) ? 32'sd0 :
        (integral_sum_q16 > INTEGRAL_LIMIT_COMMAND * 65536) ? INTEGRAL_LIMIT_COMMAND * 65536 :
        (integral_sum_q16 < -INTEGRAL_LIMIT_COMMAND * 65536) ? -INTEGRAL_LIMIT_COMMAND * 65536 : integral_sum_q16[31:0];
    wire signed [31:0] integral_output_q8 = integral_candidate_q16[31] ?
        -((-$signed(integral_candidate_q16)) >>> 8) : (integral_candidate_q16 >>> 8);
    wire signed [48:0] sum_q8 = $signed(p_hold) + $signed(d_hold) + $signed(integral_output_q8);
    wire signed [48:0] raw_command = sum_hold[48] ? -((-sum_hold) >>> 8) : (sum_hold >>> 8);
    wire signed [48:0] oriented = MOTOR_POLARITY ? -raw_hold : raw_hold;
    wire signed [31:0] limited_full = (oriented > MAX_COMMAND) ? MAX_COMMAND :
        (oriented < -MAX_COMMAND) ? -MAX_COMMAND : oriented[31:0];
    wire signed [15:0] limited = limited_full[15:0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(i=0;i<N;i=i+1) history[i] <= 0;
            history_index <= 0; history_ready <= 0; center_count <= 0; velocity_q8 <= 0; error_hold_q8 <= 0;
            p_hold <= 0; d_hold <= 0; sum_hold <= 0; raw_hold <= 0; stage <= 0;
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            command <= 0; command_update <= 0; inside_deadband <= 0; deadband_confirmed <= 0;
        end else if(!enable) begin
            for(i=0;i<N;i=i+1) history[i] <= measurement;
            history_index <= 0; history_ready <= 0; center_count <= 0; velocity_q8 <= 0; stage <= 0;
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            command <= 0; command_update <= 0; inside_deadband <= 0; deadband_confirmed <= 0;
        end else if(reset_integral) begin
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            stage <= 0; command_update <= 0;
        end else if(update) begin
            // Seed D from the first valid update, never from an inhibited ADC
            // value. P still acts immediately. This also covers enable and ADC
            // changing together when a transient angle inhibit clears.
            if (!history_ready) begin
                for(i=0;i<N;i=i+1) history[i] <= measurement;
                history_index <= 0; history_ready <= 1; velocity_q8 <= 0;
            end else begin
                history[history_index] <= measurement;
                history_index <= history_index == N-1 ? 0 : history_index + 1'b1;
                velocity_q8 <= velocity_next;
            end
            if(!center_now) center_count <= 0;
            else if(center_count < DEADBAND_CONFIRM_TICKS) center_count <= center_count + 1'b1;
            inside_deadband <= center_now; deadband_confirmed <= qualified_now;
            error_hold_q8 <= effective_error_q8;
            stage <= 1; command_update <= 0;
        end else begin
            command_update <= 0;
            case(stage)
                1: begin
                    p_hold <= p_product; d_hold <= d_limited;
                    integral_candidate_q16 <= integral_next_q16; stage <= 2;
                end
                2: begin sum_hold <= sum_q8; stage <= 3; end
                3: begin
                    raw_hold <= raw_command;
                    if(active_ki_q8 == 0 || !integral_allowed) integral_q16 <= 0;
                    else if(!((sum_hold > MAX_COMMAND * 256 && error_hold_q8 > 0) ||
                              (sum_hold < -MAX_COMMAND * 256 && error_hold_q8 < 0)))
                        integral_q16 <= integral_candidate_q16;
                    stage <= 4;
                end
                4: begin command <= limited; command_update <= 1; stage <= 0; end
                default: stage <= 0;
            endcase
        end
    end
endmodule
