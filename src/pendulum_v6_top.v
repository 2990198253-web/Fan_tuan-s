module pendulum_v6_top #(
    parameter integer KEY_DEBOUNCE_CYCLES = 1000000,
    parameter integer ADC_SAMPLE_DIV = 1250,
    parameter integer ADC_AVERAGE_LOG2 = 7,
    parameter integer TELEMETRY_DIV = 500000,
    parameter integer UART_BAUD = 115200,
    parameter integer PWM_PERIOD = 5000,
    parameter integer MOTOR_PORT = 1,
    // Fixed upright center in the ADC's original integer-count units.
    parameter integer UPRIGHT_ADC = 755,
    parameter integer ANGLE_DEADBAND_X2 = 8,
    parameter integer DEADBAND_CONFIRM_TICKS = 20,
    parameter integer ARM_WINDOW_ADC = 20,
    parameter integer TRIP_WINDOW_ADC = 200,
    parameter integer SENSOR_TIMEOUT_CYCLES = 250000,
    parameter integer CONTROL_DIV = 25000,
    parameter signed [15:0] KP_DUTY_PER_COUNT = 16'sd220,
    parameter signed [15:0] KD_DUTY_PER_COUNT = 16'sd400,
    // Stage-1 tuning: angle integral stays off until P/D are stable on hardware.
    parameter signed [15:0] KI_DUTY_PER_COUNT_Q8 = 16'sd0,
    parameter integer INTEGRAL_LIMIT_COMMAND = 200,
    parameter integer INTEGRAL_WINDOW_ADC = 10,
    parameter integer MAX_D_COMMAND = 1400,
    parameter integer DERIVATIVE_WINDOW_TICKS = 5,
    parameter integer DERIVATIVE_FILTER_SHIFT = 1,
    parameter integer MAX_COMMAND = 4000,
    parameter integer MIN_ACTIVE_COMMAND = 1200,
    // Smooth mode follows small PD commands instead of flooring them to 24%.
    // Set to 0 for the legacy limiter/P-deadband behavior (baseline is backed up).
    parameter integer SMOOTH_BALANCE_ENABLE = 1,
    // Legacy mode only: bounded output reduction after center qualification.
    parameter integer OUTPUT_RELEASE_STEP = 100,
    parameter integer START_COMMAND = 2000,
    parameter integer START_TICKS = 40,
    parameter integer MOTOR_POLARITY = 0,
    parameter integer ENABLE_MOTOR_OUTPUT = 1,
    parameter integer ANGLE_TRIP_CONFIRM_TICKS = 15,
    // Existing board-tested position P + speed feedback; I stays disabled.
    parameter integer ENABLE_POSITION_LOOP =1,
    parameter integer POSITION_DIV = 250000,
    parameter integer ENCODER_POLARITY = 0,
    // At 50 MHz require the synchronized AB pair stable for 1 us.
    parameter integer ENCODER_FILTER_CYCLES = 50,
    parameter integer POSITION_POLARITY = 1,
    parameter integer POSITION_TARGET_MODE = 0,
    parameter signed [31:0] POSITION_TARGET_COUNTS = 32'sd0,
    parameter signed [15:0] POSITION_KP_Q8 = 16'sd4,
    parameter signed [15:0] POSITION_KD_Q8 = 16'sd1790,
    parameter signed [15:0] POSITION_KI_Q16 = 16'sd0,
    parameter integer POSITION_I_LIMIT_ADC = 1,
    parameter integer POSITION_INTEGRAL_WINDOW_COUNTS = 0,
    parameter integer MAX_TARGET_OFFSET_ADC = 8,
    parameter integer MAX_TARGET_STEP_Q8 = 32,
    parameter integer POSITION_SOFT_BOUND_COUNTS = 10,
    parameter integer POSITION_SOFT_BOUND_MARGIN_COUNTS = 4,
    parameter integer POSITION_SOFT_BOUND_GAIN_Q8 = 128,
    parameter integer POSITION_SOFT_CLIP_KNEE_ADC = 0,
    parameter integer POSITION_DISABLE_GUARD_WITH_ZERO_GAINS = 0,
    parameter integer POSITION_TOLERANCE_COUNTS =2,
    parameter integer TELEMETRY_VIEW = 1,
    // Half-turn calibration: upright755 minus hanging225 -> 530 ADC counts.
    parameter integer SWING_ANGLE_PERIOD_ADC = 1060,
    parameter integer SWING_MAX_COMMAND = 1400,
    parameter integer SWING_START_COMMAND = 900,
    parameter integer SWING_RAMP_TICKS = 200,
    parameter integer SWING_KICK_TICKS = 100,
    parameter integer SWING_TIMEOUT_TICKS = 30000,
    parameter integer SWING_VELOCITY_TICKS = 20,
    parameter integer SWING_BOTTOM_SPEED_ADC = 24,
    parameter integer SWING_CAPTURE_SPEED_ADC = 4,
    parameter integer SWING_CAPTURE_ARM_SPEED_COUNTS = 4,
    parameter integer SWING_CAPTURE_SAMPLES = 2,
    parameter integer SWING_SOFT_TRAVEL_COUNTS = 150,
    parameter integer SWING_HARD_TRAVEL_COUNTS = 240,
    parameter integer ENABLE_HARD_TRAVEL_STOP = 1,
    parameter integer SWING_POLARITY = 0,
    // Measured on 2026-10-23: positive command -> increasing raw encoder.
    // This is independent of angle-loop MOTOR/POSITION_POLARITY.
    parameter integer SWING_ENCODER_DRIVE_POLARITY = 0,
    parameter integer SWING_MAX_ARM_SPEED_COUNTS = 5,
    parameter integer SWING_RESUME_ARM_SPEED_COUNTS = 2,
    parameter integer SWING_PREDICT_SAMPLES = 8,
    parameter integer SWING_RETURN_WINDOW_COUNTS = 60,
    parameter integer SWING_BRAKE_COMMAND = 1400,
    parameter integer FIRMWARE_REVISION = 6,
    parameter integer SWING_LAB_ENABLE = 0,
    parameter integer LAB_ENERGY_DEFAULT = 0,
    parameter integer LAB_RELAY_DEFAULT = 0,
    parameter integer LAB_CATCH_DEFAULT = 0,
    parameter integer LAB_REFERENCE_DEFAULT=0,LAB_REFERENCE_ENABLE=0,
    parameter integer LAB_REFERENCE_ONLY=0,
    parameter integer LAB_CLOCK_HZ = 50000000,
    parameter integer LAB_SETTLE_CYCLES = 10000000,
    // R5: full existing 80% cap, short kick then active reverse deceleration.
    parameter integer SWING_IMPULSE_ENABLE = 1,
    parameter integer SWING_IMPULSE_COMMAND = 4000,
    parameter integer SWING_IMPULSE_DISTANCE = 80,
    parameter integer SWING_IMPULSE_SAMPLES = 8,
    parameter integer SWING_IMPULSE_SPEED = 10,
    parameter integer SWING_IMPULSE_EMERGENCY_SPEED = 16,
    parameter integer SWING_TRAJECTORY_ENABLE = 1,
    parameter integer SWING_TRAJECTORY_START_STROKE = 60,
    parameter integer SWING_TRAJECTORY_MAX_STROKE = 120,
    parameter integer SWING_TRAJECTORY_STROKE_STEP = 10,
    parameter integer SWING_TRAJECTORY_MAX_SPEED = 8,
    parameter integer SWING_TRAJECTORY_EMERGENCY_SPEED = 12,
    parameter integer SWING_TRAJECTORY_POSITION_GAIN_Q8 = 24,
    parameter integer SWING_TRAJECTORY_VELOCITY_GAIN = 200,
    parameter integer SWING_TRAJECTORY_FEEDFORWARD_GAIN = 70,
    parameter integer SWING_TRAJECTORY_ACCEL_STEP_Q8 = 128,
    parameter integer SWING_TRAJECTORY_PHASE_HOLD = 8
) (
    input wire clk_50m, rst_n,
    input wire [3:0] key_sw,
    input wire [9:0] adc_data_in,
    input wire enc1_a, enc1_b, enc2_a, enc2_b,
    input wire uart_rx,
    output wire adc_clk, adc_oe_n,
    output wire AN1, AN2, PWMA, BN1, BN2, PWMB,
    output wire uart_tx
);
    wire [3:0] pressed, press_pulse;
    reg serial_enabled;
    wire emergency_stop, armed, fault_latched;
    wire swing_active, balance_mode, motion_safe, capture_balance;
    wire [2:0] auto_state;
    wire signed [15:0] swing_command;
    wire [3:0] swing_action;
    wire signed [7:0] swing_target_offset;
    wire [9:0] angle_raw;
    wire [10:0] angle_x2;
    wire angle_valid, angle_update;
    reg [31:0] sensor_age, control_count, position_count;
    wire [3:0] first_inhibit_reason;
    wire [223:0] lab_config_words;
    wire lab_reference_mode=LAB_REFERENCE_ONLY!=0 || (SWING_LAB_ENABLE!=0 && lab_config_words[9*16+7]);
    wire lab_zero_busy,lab_zero_valid,lab_zero_failed;
    wire [9:0] lab_zero_adc;
    wire [12:0] lab_zero_revision;
    wire [9:0] lab_upright_adc=LAB_REFERENCE_ENABLE!=0 && lab_reference_mode && lab_zero_valid ? lab_zero_adc : UPRIGHT_ADC;
    localparam integer LAB_MS_CYCLES=LAB_CLOCK_HZ/1000>0 ? LAB_CLOCK_HZ/1000 : 1;
    reg [31:0] lab_ms_count;
    wire lab_ms_tick=lab_ms_count==LAB_MS_CYCLES-1;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)lab_ms_count<=0;else if(lab_ms_tick)lab_ms_count<=0;else lab_ms_count<=lab_ms_count+1'b1;
    end
    wire lab_remote_stop;
    wire lab_packet_seen;reg [31:0] lab_link_age;reg lab_link_fault;
    wire lab_link_lost=SWING_LAB_ENABLE!=0 && lab_link_age>=LAB_CLOCK_HZ;
    wire stop_request=!key_sw[1] || press_pulse[1] || (SWING_LAB_ENABLE!=0 && lab_remote_stop) || (armed && lab_link_lost);
    wire control_tick = control_count == CONTROL_DIV - 1;
    wire position_tick = position_count == POSITION_DIV - 1;
    wire sensor_fresh = angle_valid && sensor_age < SENSOR_TIMEOUT_CYCLES;
    wire signed [31:0] angle_error_x2 = $signed({1'b0,angle_x2}) - $signed({1'b0,lab_upright_adc}) * 2;
    wire signed [31:0] abs_angle_error_x2 = angle_error_x2[31] ? -angle_error_x2 : angle_error_x2;
    wire in_trip_window = abs_angle_error_x2 <= TRIP_WINDOW_ADC * 2;
    wire balance_active = armed && balance_mode && sensor_fresh && !fault_latched && !emergency_stop && motion_safe;
    wire safe_active = balance_active && in_trip_window && key_sw[1] && key_sw[3];
    wire lab_target_safe = safe_active && abs_angle_error_x2 <= 32 && !lab_link_lost;
    // Stop events dominate an arm event even if the raw stop pin has just
    // returned high on the debounce-pulse cycle.
    wire arm_accept = press_pulse[0] && !stop_request && !press_pulse[3] &&
        !armed && key_sw[1] && key_sw[3] &&
        !emergency_stop && !fault_latched && sensor_fresh && !lab_link_lost &&
        (!lab_reference_mode || LAB_REFERENCE_ENABLE==0 || (lab_zero_valid && !lab_zero_busy));
    wire drive_safe = armed && sensor_fresh && !fault_latched && !emergency_stop && motion_safe &&
        key_sw[1] && key_sw[3] && !(SWING_LAB_ENABLE!=0 && lab_remote_stop) && !lab_link_lost && (swing_active || (balance_mode && in_trip_window));
    reg safe_active_q,drive_safe_q;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)begin safe_active_q<=0;drive_safe_q<=0;end
        else begin safe_active_q<=safe_active;drive_safe_q<=drive_safe;end
    end
    wire signed [31:0] encoder1, encoder2;
    wire encoder1_illegal, encoder2_illegal;
    wire signed [31:0] selected_encoder = MOTOR_PORT == 2 ? encoder2 : encoder1;
    wire signed [31:0] position_feedback_raw = ENCODER_POLARITY ? -selected_encoder : selected_encoder;
    reg signed [31:0] position_feedback;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)position_feedback<=0;
        else position_feedback<=position_feedback_raw;
    end
    lab_zero_calibrator #(.DEFAULT_ADC(UPRIGHT_ADC)) zero_calibrator(
        .clk(clk_50m),.rst_n(rst_n),.enable(LAB_REFERENCE_ENABLE!=0 && lab_reference_mode),
        .idle(!armed && !fault_latched && !emergency_stop && key_sw[1] && key_sw[3] && sensor_fresh),
        .update_ms(lab_ms_tick),.request(press_pulse[2]),.angle_x2(angle_x2),.position(position_feedback),
        .busy(lab_zero_busy),.valid(lab_zero_valid),.failed(lab_zero_failed),
        .zero_adc(lab_zero_adc),.revision(lab_zero_revision));
    wire signed [31:0] position_offset_q8, position_target, position_anchor, position_velocity_q8;
    reg [15:0] position_kp_live,position_ki_live,position_kd_live;
    reg [15:0] inner_kp_live,inner_ki_live,inner_kd_live;
    wire signed [32:0] position_error;
    reg signed [15:0] position_error_sampled;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)position_error_sampled<=0;
        else if(position_error>33'sd32767)position_error_sampled<=16'sd32767;
        else if(position_error< -33'sd32768)position_error_sampled<= -16'sd32768;
        else position_error_sampled<=position_error[15:0];
    end
    wire signed [32:0] abs_position_error = position_error[32] ? -position_error : position_error;
    wire signed [31:0] abs_position_velocity = position_velocity_q8[31] ? -position_velocity_q8 : position_velocity_q8;
    // A speed-only loop must not request minimum drive for a static displacement.
    wire position_needs_action = ENABLE_POSITION_LOOP != 0 &&
        (((position_kp_live != 0 || position_ki_live != 0) &&
          abs_position_error > POSITION_TOLERANCE_COUNTS) ||
         (position_kd_live != 0 && abs_position_velocity >= 256));
    wire signed [15:0] pd_command, compensated_command;
    wire pd_command_update, inside_deadband, deadband_confirmed;
    reg signed [15:0] swing_command_q;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)swing_command_q<=0;
        else swing_command_q<=swing_command;
    end
    wire signed [15:0] selected_command = swing_active ? swing_command_q : compensated_command;
    wire signed [15:0] bounded_command = selected_command > MAX_COMMAND ? MAX_COMMAND :
        selected_command < -MAX_COMMAND ? -MAX_COMMAND : selected_command;
    wire signed [15:0] motor_command = drive_safe ? bounded_command : 16'sd0;
    reg signed [15:0] motor_command_q;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)motor_command_q<=0;
        // The registered command uses the already registered enable. Raw
        // drive_safe still gates motor pins directly on a stop/fault edge.
        else motor_command_q<=drive_safe_q ? bounded_command : 16'sd0;
    end
    wire motor_in1, motor_in2, motor_pwm;
    wire [7:0] telemetry_status = {(MOTOR_PORT == 2),encoder2_illegal,encoder1_illegal,
        angle_valid,emergency_stop,fault_latched,armed,serial_enabled};
    // V6 auxiliary: bit31=swinging, bits30:27=stop/fault reason,
    // bits26:11=signed outer target Q8, bits10:0=half-count ADC.
    // SWING: R4=0xD<action><target>; R5=0xE<action><reserved zero>.
    // Outside SWING the original signed outer-target Q8 field is unchanged.
    wire [3:0] swing_debug_tag=SWING_IMPULSE_ENABLE!=0 ? 4'hE : 4'hD;
    wire [15:0] debug_payload=swing_active ? {swing_debug_tag,swing_action,swing_target_offset} : position_offset_q8[15:0];
    wire [31:0] debug_word = {swing_active,first_inhibit_reason,debug_payload,angle_x2};
    wire [7:0] lab_rx_byte,lab_ack_seq,lab_ack_status;
    wire lab_rx_valid,lab_rx_error;
    wire [15:0] lab_revision,lab_crc_errors;
    wire [3:0] lab_cut_reason;
    wire [15:0] lab_step_ms,lab_last_accel_ms,lab_pulse_index,lab_run_ms;
    wire signed [6:0] lab_arm_speed;
    wire signed [15:0] lab_energy_proxy;
    wire signed [15:0] lab_relay_delta;
    wire [3:0] lab_capture_diag;
    wire lab_reference_qualified;
    reg [31:0] lab_settle_count;reg signed [31:0] lab_previous_position;
    reg [15:0] lab_trial_id;
    wire lab_config_safe=!armed && !fault_latched && !emergency_stop &&
        key_sw[1] && key_sw[3] && sensor_fresh && lab_settle_count>=LAB_SETTLE_CYCLES && !arm_accept && !lab_zero_busy;
    wire lab_tune_safe = lab_config_safe || lab_target_safe;
    reg lab_tune_safe_q;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)lab_tune_safe_q<=0;
        else lab_tune_safe_q<=lab_tune_safe;
    end
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)begin lab_settle_count<=0;lab_previous_position<=0;lab_trial_id<=0;lab_link_age<=LAB_CLOCK_HZ;lab_link_fault<=0;end
        else begin
            if(lab_packet_seen)lab_link_age<=0;else if(lab_link_age<LAB_CLOCK_HZ)lab_link_age<=lab_link_age+1'b1;
            if(arm_accept)lab_link_fault<=0;
            else if(armed && lab_link_lost)lab_link_fault<=1;
            lab_previous_position<=position_feedback;
            if(armed || position_feedback!=lab_previous_position)lab_settle_count<=0;
            else if(lab_settle_count<LAB_SETTLE_CYCLES)lab_settle_count<=lab_settle_count+1'b1;
            if(arm_accept)lab_trial_id<=lab_trial_id+1'b1;
        end
    end
    lab_uart_rx #(.CLOCK_HZ(LAB_CLOCK_HZ),.BAUD(UART_BAUD)) lab_receiver(
        .clk(clk_50m),.rst_n(rst_n),.rx(uart_rx),.data(lab_rx_byte),.valid(lab_rx_valid),.frame_error(lab_rx_error));
    wire lab_target_request;
    wire signed [15:0] lab_target_relative;
    wire [15:0] lab_target_speed;
    wire lab_gain_request,lab_gain_i_present,lab_inner_gain_request;
    wire [15:0] lab_gain_p_q8,lab_gain_i_q16,lab_gain_d_q8;
    wire [15:0] lab_inner_p,lab_inner_i_q8,lab_inner_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)begin position_kp_live<=POSITION_KP_Q8;position_ki_live<=POSITION_KI_Q16;position_kd_live<=POSITION_KD_Q8;end
        else if(lab_gain_request)begin
            position_kp_live<=lab_gain_p_q8;position_kd_live<=lab_gain_d_q8;
            if(lab_gain_i_present)position_ki_live<=lab_gain_i_q16;
        end
    end
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)begin inner_kp_live<=KP_DUTY_PER_COUNT;inner_ki_live<=KI_DUTY_PER_COUNT_Q8;inner_kd_live<=KD_DUTY_PER_COUNT;end
        else if(lab_inner_gain_request)begin
            inner_kp_live<=lab_inner_p;inner_ki_live<=lab_inner_i_q8;inner_kd_live<=lab_inner_d;
        end
    end
    lab_config #(.CLOCK_HZ(LAB_CLOCK_HZ),.ENERGY_DEFAULT(LAB_ENERGY_DEFAULT),.RELAY_DEFAULT(LAB_RELAY_DEFAULT),.CATCH_DEFAULT(LAB_CATCH_DEFAULT),.REFERENCE_DEFAULT(LAB_REFERENCE_DEFAULT),.REFERENCE_ONLY(LAB_REFERENCE_ONLY)) lab_parameters(
        .clk(clk_50m),.rst_n(rst_n),.byte_valid(lab_rx_valid && SWING_LAB_ENABLE!=0),
        .frame_error(lab_rx_error),.safe_to_config(lab_config_safe),.safe_to_target(lab_target_safe),
        .safe_to_tune(lab_tune_safe_q),.byte_data(lab_rx_byte),
        .config_words(lab_config_words),.ack_sequence(lab_ack_seq),.ack_status(lab_ack_status),
        .revision(lab_revision),.crc_errors(lab_crc_errors),.remote_stop(lab_remote_stop),.packet_seen(lab_packet_seen),
        .target_request(lab_target_request),.target_relative(lab_target_relative),.target_speed(lab_target_speed),
        .gain_request(lab_gain_request),.gain_i_present(lab_gain_i_present),
        .gain_p_q8(lab_gain_p_q8),.gain_i_q16(lab_gain_i_q16),.gain_d_q8(lab_gain_d_q8),
        .inner_gain_request(lab_inner_gain_request),.inner_p(lab_inner_p),
        .inner_i_q8(lab_inner_i_q8),.inner_d(lab_inner_d));
    reg [3:0] swing_action_sampled;
    always @(posedge clk_50m or negedge rst_n)begin
        if(!rst_n)swing_action_sampled<=0;
        else swing_action_sampled<=swing_action;
    end
    reg [495:0] lab_packet;
    // During BALANCE, telemetry word 20 reports the angle target in half-ADC
    // counts. It includes the position loop's live target correction.
    wire signed [31:0] lab_inner_target_x2 = $signed({1'b0,lab_upright_adc,1'b0}) +
        (ENABLE_POSITION_LOOP != 0 ? (position_offset_q8 >>> 7) : 32'sd0);
    always @*begin
        lab_packet=0;
        lab_packet[0*8+:8]=8'hA5;lab_packet[1*8+:8]=8'h5A;
        // 0x65 also reports the active inner angle target while balancing.
        lab_packet[2*8+:8]=LAB_REFERENCE_ONLY!=0 ? 8'h65 : 8'h61;
        lab_packet[4*8+:8]=telemetry_status;lab_packet[5*8+:8]={5'd0,auto_state};
        lab_packet[6*8+:8]={4'd0,swing_action_sampled};
        lab_packet[7*8+:8]={4'd0,(lab_capture_diag!=0 ? lab_capture_diag : lab_cut_reason)};
        lab_packet[8*8+:8]=lab_link_fault && first_inhibit_reason==1 ? 8'd10 : {4'd0,first_inhibit_reason};lab_packet[9*8+:8]=lab_ack_seq;
        lab_packet[10*8+:8]=lab_ack_status;lab_packet[11*8+:8]={lab_arm_speed,lab_config_safe};
        lab_packet[12*8+:16]={5'd0,angle_x2};lab_packet[14*8+:32]=selected_encoder;
        lab_packet[18*8+:16]=motor_command_q;
        lab_packet[20*8+:16]=lab_reference_mode && balance_mode ? lab_inner_target_x2[15:0] :
            lab_reference_mode ? {6'd0,lab_upright_adc} : lab_step_ms;
        lab_packet[22*8+:16]=lab_reference_mode ? {lab_zero_valid,lab_zero_busy,lab_zero_failed,lab_zero_revision} : lab_pulse_index;lab_packet[24*8+:16]=lab_revision;
        lab_packet[26*8+:224]=lab_config_words;lab_packet[54*8+:16]=lab_trial_id;
        lab_packet[56*8+:16]=lab_crc_errors;lab_packet[58*8+:16]=lab_run_ms;
        lab_packet[60*8+:16]=lab_reference_mode && balance_mode ? position_error_sampled :
            lab_reference_mode || lab_config_words[9*16+5] ? lab_relay_delta :
            lab_config_words[9*16+4] ? lab_energy_proxy : lab_last_accel_ms;
    end
    wire lab_tx,legacy_tx;
    lab_telemetry #(.CLOCK_HZ(LAB_CLOCK_HZ),.BAUD(UART_BAUD),.FRAME_DIV(TELEMETRY_DIV)) lab_sender(
        .clk(clk_50m),.rst_n(rst_n),.enable(SWING_LAB_ENABLE!=0),.packet(lab_packet),.tx(lab_tx));
    assign uart_tx=SWING_LAB_ENABLE!=0 ? lab_tx : legacy_tx;

    genvar i;
    generate for(i=0;i<4;i=i+1) begin: KEYS
        key_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) filter (
            .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[i]),.pressed(pressed[i]),.press_pulse(press_pulse[i]));
    end endgenerate

    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            serial_enabled <= 0;
            sensor_age <= SENSOR_TIMEOUT_CYCLES; control_count <= 0;
            position_count <= 0;
        end else begin
            if(control_tick) control_count <= 0;
            else control_count <= control_count + 1'b1;
            if(!safe_active_q || position_tick) position_count <= 0;
            else position_count <= position_count + 1'b1;
            if(angle_update) sensor_age <= 0;
            else if(sensor_age < SENSOR_TIMEOUT_CYCLES) sensor_age <= sensor_age + 1'b1;
            if(press_pulse[2] || arm_accept) serial_enabled <= 1;
            if(arm_accept || capture_balance) begin
                control_count <= CONTROL_DIV - 1; position_count <= 0;
            end
        end
    end

    swing_up_supervisor #(.UPRIGHT_ADC(UPRIGHT_ADC),.ANGLE_PERIOD_ADC(SWING_ANGLE_PERIOD_ADC),
        .CAPTURE_WINDOW_ADC(ARM_WINDOW_ADC),.CAPTURE_SPEED_ADC(SWING_CAPTURE_SPEED_ADC),
        .CAPTURE_ARM_SPEED_COUNTS(SWING_CAPTURE_ARM_SPEED_COUNTS),
        .CAPTURE_CONFIRM_SAMPLES(SWING_CAPTURE_SAMPLES),.VELOCITY_TICKS(SWING_VELOCITY_TICKS),
        .BOTTOM_SPEED_ADC(SWING_BOTTOM_SPEED_ADC),.MAX_COMMAND(SWING_MAX_COMMAND),
        .START_COMMAND(SWING_START_COMMAND),.RAMP_TICKS(SWING_RAMP_TICKS),
        .KICK_TICKS(SWING_KICK_TICKS),.TIMEOUT_TICKS(SWING_TIMEOUT_TICKS),
        .SOFT_TRAVEL_COUNTS(SWING_SOFT_TRAVEL_COUNTS),.HARD_TRAVEL_COUNTS(SWING_HARD_TRAVEL_COUNTS),
        .ENABLE_HARD_TRAVEL_STOP(ENABLE_HARD_TRAVEL_STOP),
        .TRIP_WINDOW_ADC(TRIP_WINDOW_ADC),.TRIP_CONFIRM_TICKS(ANGLE_TRIP_CONFIRM_TICKS),
        .DRIVE_POLARITY(MOTOR_POLARITY ^ SWING_POLARITY),
        .RETURN_POLARITY(SWING_ENCODER_DRIVE_POLARITY ^ ENCODER_POLARITY),
        .BALANCE_POLARITY(MOTOR_POLARITY),
        .LIVE_UPRIGHT_ENABLE(LAB_REFERENCE_ENABLE),
        .MAX_ARM_SPEED_COUNTS(SWING_MAX_ARM_SPEED_COUNTS),
        .RESUME_ARM_SPEED_COUNTS(SWING_RESUME_ARM_SPEED_COUNTS),
        .PREDICT_SAMPLES(SWING_PREDICT_SAMPLES),.RETURN_WINDOW_COUNTS(SWING_RETURN_WINDOW_COUNTS),
        .BRAKE_COMMAND(SWING_BRAKE_COMMAND),.LAB_ENABLE(SWING_LAB_ENABLE),.REFERENCE_ONLY(LAB_REFERENCE_ONLY),.IMPULSE_ENABLE(SWING_IMPULSE_ENABLE),
        .IMPULSE_COMMAND(SWING_IMPULSE_COMMAND),.IMPULSE_DISTANCE(SWING_IMPULSE_DISTANCE),
        .IMPULSE_SAMPLES(SWING_IMPULSE_SAMPLES),.IMPULSE_SPEED(SWING_IMPULSE_SPEED),
        .IMPULSE_EMERGENCY_SPEED(SWING_IMPULSE_EMERGENCY_SPEED),.TRAJECTORY_ENABLE(SWING_TRAJECTORY_ENABLE),
        .TRAJECTORY_START_STROKE(SWING_TRAJECTORY_START_STROKE),.TRAJECTORY_MAX_STROKE(SWING_TRAJECTORY_MAX_STROKE),
        .TRAJECTORY_STROKE_STEP(SWING_TRAJECTORY_STROKE_STEP),.TRAJECTORY_MAX_SPEED(SWING_TRAJECTORY_MAX_SPEED),
        .TRAJECTORY_EMERGENCY_SPEED(SWING_TRAJECTORY_EMERGENCY_SPEED),
        .TRAJECTORY_POSITION_GAIN_Q8(SWING_TRAJECTORY_POSITION_GAIN_Q8),
        .TRAJECTORY_VELOCITY_GAIN(SWING_TRAJECTORY_VELOCITY_GAIN),
        .TRAJECTORY_FEEDFORWARD_GAIN(SWING_TRAJECTORY_FEEDFORWARD_GAIN),
        .TRAJECTORY_ACCEL_STEP_Q8(SWING_TRAJECTORY_ACCEL_STEP_Q8),.TRAJECTORY_PHASE_HOLD(SWING_TRAJECTORY_PHASE_HOLD)) swing_controller (
        .clk(clk_50m),.rst_n(rst_n),.update(control_tick),.start(arm_accept),
        .stop(stop_request),.estop(!key_sw[3] || press_pulse[3]),
        .lab_config_words(lab_config_words),.lab_cut_reason(lab_cut_reason),.lab_step_ms(lab_step_ms),
        .lab_last_accel_ms(lab_last_accel_ms),.lab_pulse_index(lab_pulse_index),.lab_run_ms(lab_run_ms),
        .lab_arm_speed(lab_arm_speed),
        .lab_energy_proxy(lab_energy_proxy),
        .lab_relay_delta(lab_relay_delta),.lab_capture_diag(lab_capture_diag),.reference_observation_valid(lab_reference_qualified),
        .sensor_fresh(sensor_fresh),.encoder_illegal(MOTOR_PORT==2 ? encoder2_illegal : encoder1_illegal),
        .angle_x2(angle_x2),.upright_adc(lab_upright_adc),.reference_update_ms(lab_ms_tick),.position(position_feedback),.running(armed),
        .swinging(swing_active),.balancing(balance_mode),.fault(fault_latched),.emergency(emergency_stop),
        .motion_safe(motion_safe),.capture_balance(capture_balance),.reason(first_inhibit_reason),
        .swing_command(swing_command),.state(auto_state),.swing_action(swing_action),.swing_target_offset(swing_target_offset));

    adc_capture #(.SAMPLE_DIV(ADC_SAMPLE_DIV),.AVERAGE_LOG2(ADC_AVERAGE_LOG2)) adc (
        .clk(clk_50m),.rst_n(rst_n),.adc_data(adc_data_in),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),
        .angle_raw(angle_raw),.angle_x2(angle_x2),.angle_valid(angle_valid),.angle_update(angle_update));
    quadrature_counter #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES)) encoder_a (.clk(clk_50m),.rst_n(rst_n),.phase_a(enc1_a),.phase_b(enc1_b),
        .position(encoder1),.illegal_seen(encoder1_illegal));
    quadrature_counter #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES)) encoder_b (.clk(clk_50m),.rst_n(rst_n),.phase_a(enc2_a),.phase_b(enc2_b),
        .position(encoder2),.illegal_seen(encoder2_illegal));

    position_pid_controller #(.KP_Q8(POSITION_KP_Q8),.KD_Q8(POSITION_KD_Q8),.KI_Q16(POSITION_KI_Q16),.RUNTIME_GAINS(1),
        .I_LIMIT_ADC(POSITION_I_LIMIT_ADC),.INTEGRAL_WINDOW_COUNTS(POSITION_INTEGRAL_WINDOW_COUNTS),
        .MAX_OFFSET_ADC(MAX_TARGET_OFFSET_ADC),.MAX_OFFSET_STEP_Q8(MAX_TARGET_STEP_Q8),
        .SOFT_BOUND_COUNTS(POSITION_SOFT_BOUND_COUNTS),
        .SOFT_BOUND_MARGIN_COUNTS(POSITION_SOFT_BOUND_MARGIN_COUNTS),
        .SOFT_BOUND_GAIN_Q8(POSITION_SOFT_BOUND_GAIN_Q8),
        .SOFT_CLIP_KNEE_ADC(POSITION_SOFT_CLIP_KNEE_ADC),
        .DISABLE_GUARD_WITH_ZERO_GAINS(POSITION_DISABLE_GUARD_WITH_ZERO_GAINS),
        .VELOCITY_FILTER_SHIFT(1),.TARGET_MODE(POSITION_TARGET_MODE),.TARGET_COUNTS(POSITION_TARGET_COUNTS),
        .OUTPUT_POLARITY(POSITION_POLARITY ^ MOTOR_POLARITY)) position_controller (
        .clk(clk_50m),.rst_n(rst_n),.enable(safe_active_q && ENABLE_POSITION_LOOP != 0),
        .runtime_kp_q8(position_kp_live),.runtime_ki_q16(position_ki_live),
        .runtime_kd_q8(position_kd_live),.reset_integral(lab_gain_request && lab_gain_i_present),
        .capture(capture_balance),.update(position_tick && safe_active_q && ENABLE_POSITION_LOOP != 0),
        .move_request(lab_target_request),.move_relative(lab_target_relative),.move_speed(lab_target_speed),
        .position(position_feedback),.target_offset_q8(position_offset_q8),.target_position(position_target),
        .anchor_position(position_anchor),
        .position_error(position_error),.velocity_q8(position_velocity_q8));
    wire lab_pid_warm=LAB_REFERENCE_ENABLE!=0 && lab_reference_mode && swing_active && lab_zero_valid &&
        sensor_fresh && in_trip_window && !fault_latched && !emergency_stop && key_sw[1] && key_sw[3] &&
        lab_reference_qualified;
    // Isolate the long reference-qualification cone from the PID DSP input.
    // Motor pins remain gated by drive_safe without this extra clock.
    reg lab_pid_warm_q;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) lab_pid_warm_q <= 1'b0;
        else lab_pid_warm_q <= lab_pid_warm;
    end
    balance_pd_controller #(.UPRIGHT_ADC_X2(UPRIGHT_ADC * 2),.LIVE_UPRIGHT_ENABLE(LAB_REFERENCE_ENABLE),
        .CENTER_P_SUPPRESS(SMOOTH_BALANCE_ENABLE == 0),
        .ANGLE_DEADBAND_X2(ANGLE_DEADBAND_X2),
        .DEADBAND_CONFIRM_TICKS(DEADBAND_CONFIRM_TICKS),.CONTROL_DIV(CONTROL_DIV),
        .KP(KP_DUTY_PER_COUNT),.KD(KD_DUTY_PER_COUNT),.KI_Q8(KI_DUTY_PER_COUNT_Q8),.RUNTIME_GAINS(1),
        .INTEGRAL_LIMIT_COMMAND(INTEGRAL_LIMIT_COMMAND),.INTEGRAL_WINDOW_ADC(INTEGRAL_WINDOW_ADC),
        .MAX_D_COMMAND(MAX_D_COMMAND),
        .MAX_COMMAND(MAX_COMMAND),.DERIVATIVE_WINDOW_TICKS(DERIVATIVE_WINDOW_TICKS),
        .DERIVATIVE_FILTER_SHIFT(DERIVATIVE_FILTER_SHIFT),.MOTOR_POLARITY(MOTOR_POLARITY)) controller (
        // A transient inhibit must discard in-flight control/history as well
        // as gate the pins. Resume only after a fresh valid control update.
        .clk(clk_50m),.rst_n(rst_n),.enable(safe_active_q || lab_pid_warm_q),.update(control_tick && (safe_active_q || lab_pid_warm_q)),
        .runtime_kp(inner_kp_live),.runtime_ki_q8(inner_ki_live),.runtime_kd(inner_kd_live),
        .reset_integral(lab_inner_gain_request),
        .angle_x2(angle_x2),.upright_adc_x2({lab_upright_adc,1'b0}),.target_offset_q8(ENABLE_POSITION_LOOP != 0 ? position_offset_q8 : 32'sd0),
        .command(pd_command),.command_update(pd_command_update),
        .inside_deadband(inside_deadband),.deadband_confirmed(deadband_confirmed));
    balance_pwm_limiter #(.MAX_COMMAND(MAX_COMMAND),.MIN_ACTIVE_COMMAND(MIN_ACTIVE_COMMAND),
        .SMOOTH_BALANCE_ENABLE(SMOOTH_BALANCE_ENABLE),
        .OUTPUT_RELEASE_STEP(OUTPUT_RELEASE_STEP),.START_COMMAND(START_COMMAND),.START_TICKS(START_TICKS)) output_limiter (
        .clk(clk_50m),.rst_n(rst_n),.enable(safe_active_q),.update(pd_command_update),
        .inside_deadband(inside_deadband),.deadband_confirmed(deadband_confirmed),
        .position_needs_action(position_needs_action),.request(pd_command),.command(compensated_command));
    motor_signed_pwm #(.PWM_PERIOD(PWM_PERIOD),.REVERSE_BLANK_CYCLES(PWM_PERIOD)) motor (
        .clk(clk_50m),.rst_n(rst_n),.drive_enable(ENABLE_MOTOR_OUTPUT != 0 && drive_safe_q),
        .fault(fault_latched || emergency_stop),.command(motor_command_q),
        .in1(motor_in1),.in2(motor_in2),.pwm(motor_pwm));

    // Raw safety signals still inhibit every motor pin without a clock delay.
    assign AN1 = MOTOR_PORT == 2 && drive_safe ? motor_in1 : 1'b0;
    assign AN2 = MOTOR_PORT == 2 && drive_safe ? motor_in2 : 1'b0;
    assign PWMA = MOTOR_PORT == 2 && drive_safe ? motor_pwm : 1'b0;
    // J280 schematic: BN1=AA17, BN2=V15.  The supplied R1 CST had these
    // names reversed; swap the logical outputs to retain R1's physical drive
    // polarity while using the schematic-correct pin names.
    assign BN1 = MOTOR_PORT == 1 && drive_safe ? motor_in2 : 1'b0;
    assign BN2 = MOTOR_PORT == 1 && drive_safe ? motor_in1 : 1'b0;
    assign PWMB = MOTOR_PORT == 1 && drive_safe ? motor_pwm : 1'b0;
    telemetry_v1 #(.FRAME_DIV(TELEMETRY_DIV),.BAUD(UART_BAUD)) telemetry (
        .clk(clk_50m),.rst_n(rst_n),.enable(serial_enabled),.status(telemetry_status),.angle_raw(angle_raw),
        .encoder1(TELEMETRY_VIEW != 0 && MOTOR_PORT == 2 ? $signed(debug_word) : encoder1),
        .encoder2(TELEMETRY_VIEW != 0 && MOTOR_PORT == 1 ? $signed(debug_word) : encoder2),
        .motor_command(motor_command),.uart_tx_pin(legacy_tx));
endmodule
