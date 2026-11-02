// Hybrid energy-pumping swing-up with bounded travel, timeout and latched faults.
// ADC units are calibrated counts, velocity is half-counts per VELOCITY_TICKS.
// Energy is a normalized proxy, not an identified full Furuta plant model.
module swing_up_supervisor #(
    parameter integer UPRIGHT_ADC = 755,
    parameter integer ANGLE_PERIOD_ADC = 1060,
    parameter integer CAPTURE_WINDOW_ADC = 20,
    parameter integer CAPTURE_SPEED_ADC = 4,
    parameter integer CAPTURE_ARM_SPEED_COUNTS = 4,
    parameter integer CAPTURE_CONFIRM_SAMPLES = 2,
    parameter integer VELOCITY_TICKS = 20,
    parameter integer BOTTOM_SPEED_ADC = 24,
    parameter integer MAX_COMMAND = 1400,
    parameter integer START_COMMAND = 900,
    parameter integer RAMP_TICKS = 200,
    parameter integer KICK_TICKS = 100,
    parameter integer TIMEOUT_TICKS = 30000,
    parameter integer SOFT_TRAVEL_COUNTS = 150,
    parameter integer HARD_TRAVEL_COUNTS = 240,
    parameter integer ENABLE_HARD_TRAVEL_STOP = 1,
    parameter integer TRIP_WINDOW_ADC = 200,
    parameter integer TRIP_CONFIRM_TICKS = 15,
    parameter integer DRIVE_POLARITY = 0,
    // 0: positive physical command increases normalized encoder position.
    parameter integer RETURN_POLARITY = 0,
    parameter integer BALANCE_POLARITY = 0,
    parameter integer LIVE_UPRIGHT_ENABLE=0,
    parameter integer PRECAPTURE_WINDOW_ADC=100,PRECAPTURE_EXIT_ADC=140,PRECAPTURE_TIMEOUT_TICKS=800,
    parameter integer MAX_ARM_SPEED_COUNTS = 5,
    parameter integer RESUME_ARM_SPEED_COUNTS = 2,
    parameter integer PREDICT_SAMPLES = 8,
    parameter integer RETURN_WINDOW_COUNTS = 60,
    parameter integer BRAKE_COMMAND = 1400,
    parameter integer LAB_ENABLE = 0,
    parameter integer REFERENCE_ONLY=0,
    parameter integer IMPULSE_ENABLE = 0,
    parameter integer IMPULSE_COMMAND = 4000,
    parameter integer IMPULSE_DISTANCE = 80,
    parameter integer IMPULSE_SAMPLES = 8,
    parameter integer IMPULSE_SPEED = 10,
    parameter integer IMPULSE_EMERGENCY_SPEED = 16,
    parameter integer TRAJECTORY_ENABLE = 0,
    parameter integer TRAJECTORY_START_STROKE = 60,
    parameter integer TRAJECTORY_MAX_STROKE = 120,
    parameter integer TRAJECTORY_STROKE_STEP = 10,
    parameter integer TRAJECTORY_MAX_SPEED = 8,
    parameter integer TRAJECTORY_EMERGENCY_SPEED = 12,
    parameter integer TRAJECTORY_POSITION_GAIN_Q8 = 24,
    parameter integer TRAJECTORY_VELOCITY_GAIN = 200,
    parameter integer TRAJECTORY_FEEDFORWARD_GAIN = 70,
    parameter integer TRAJECTORY_ACCEL_STEP_Q8 = 128,
    parameter integer TRAJECTORY_PHASE_HOLD = 8
) (
    input wire clk, rst_n, update, start, stop, estop,
    input wire sensor_fresh, encoder_illegal,
    input wire [10:0] angle_x2,
    input wire [9:0] upright_adc,
    input wire reference_update_ms,
    input wire [223:0] lab_config_words,
    input wire signed [31:0] position,
    output wire running, swinging, balancing, fault, emergency,
    output wire motion_safe,
    output reg capture_balance,
    output reg [3:0] reason,
    output wire signed [15:0] swing_command,
    output reg [2:0] state,
    output wire [3:0] swing_action,
    output wire signed [7:0] swing_target_offset,
    output wire [3:0] lab_cut_reason,
    output wire [15:0] lab_step_ms,lab_last_accel_ms,lab_pulse_index,
    output wire [15:0] lab_run_ms,
    output wire signed [6:0] lab_arm_speed,
    output wire signed [15:0] lab_energy_proxy,
    output wire signed [15:0] lab_relay_delta,
    output wire [3:0] lab_capture_diag,
    output wire reference_observation_valid
);
    localparam [2:0] IDLE=0, SWING=1, BALANCE=2, FAULT=3, ESTOP=4, PRECAPTURE=5;
    reg [31:0] catch_age;
    reg catch_armed;
    wire catch_observation_valid,catch_boundary;
    wire signed [15:0] catch_command,catch_pole_delta;
    wire reference_mode=REFERENCE_ONLY!=0 || (LAB_ENABLE!=0 && lab_config_words[9*16+7]);
    wire [9:0] live_upright=LIVE_UPRIGHT_ENABLE!=0 ? upright_adc : UPRIGHT_ADC;
    wire signed [15:0] reference_command,reference_speed;
    wire reference_qualified,reference_capture;
    assign reference_observation_valid=reference_qualified;
    wire [1:0] reference_fault;
    wire [3:0] reference_action,reference_diagnostic;
    localparam integer PERIOD_X2 = ANGLE_PERIOD_ADC * 2;
    localparam integer HALF_X2 = ANGLE_PERIOD_ADC;
    localparam integer BOTTOM_X2 = BOTTOM_SPEED_ADC > 0 ? BOTTOM_SPEED_ADC * 2 : 1;
    localparam integer KINETIC_SCALE_Q16 = (512*65536)/(BOTTOM_X2*BOTTOM_X2);
    localparam integer RAMP_DIV = RAMP_TICKS>0 ? RAMP_TICKS : 1;
    localparam integer RAMP_RANGE = MAX_COMMAND>START_COMMAND ? MAX_COMMAND-START_COMMAND : 0;
    localparam integer RAMP_BASE = RAMP_RANGE/RAMP_DIV;
    localparam integer RAMP_REMAINDER = RAMP_RANGE%RAMP_DIV;
    reg signed [31:0] origin, previous_position;
    reg signed [32:0] arm_velocity_counts;
    reg braking, returning;
    reg signed [15:0] previous_angle, velocity_x2, drive_hold;
    reg [31:0] velocity_count, elapsed, capture_count, trip_count;
    reg [31:0] ramp_level, ramp_fraction;
    reg velocity_valid;
    reg signed [31:0] energy_error_q8;
    assign lab_energy_proxy=REFERENCE_ONLY!=0 ? 16'sd0 : energy_error_q8[15:0];
    wire signed [31:0] raw_error = $signed({1'b0,angle_x2}) - $signed({1'b0,live_upright}) * 2;
    wire signed [31:0] wrapped_error = raw_error > HALF_X2 ? raw_error-PERIOD_X2 :
        raw_error < -HALF_X2 ? raw_error+PERIOD_X2 : raw_error;
    wire [31:0] abs_error = wrapped_error[31] ? -wrapped_error : wrapped_error;
    wire signed [31:0] raw_delta = $signed({1'b0,angle_x2}) - previous_angle;
    wire signed [31:0] wrapped_delta = raw_delta > HALF_X2 ? raw_delta-PERIOD_X2 :
        raw_delta < -HALF_X2 ? raw_delta+PERIOD_X2 : raw_delta;
    wire [31:0] abs_delta = wrapped_delta[31] ? -wrapped_delta : wrapped_delta;
    wire sample_tick = update && velocity_count >= VELOCITY_TICKS-1;
    localparam integer RELAY_HALF_TICKS=VELOCITY_TICKS>1 ? VELOCITY_TICKS/2 : 1;
    wire relay_tick=update && (sample_tick || velocity_count==RELAY_HALF_TICKS-1);
    wire near_upright = abs_error <= (LAB_ENABLE!=0 ? lab_config_words[13*16+:16]*2 : CAPTURE_WINDOW_ADC*2);
    wire capture_enabled=LAB_ENABLE==0 || lab_config_words[9*16];
    assign lab_run_ms=elapsed[16:1];
    assign lab_arm_speed=arm_velocity_counts>63 ? 7'sd63 : arm_velocity_counts < -64 ? 7'sh40 : arm_velocity_counts[6:0];
    wire signed [32:0] arm_delta = $signed({position[31],position}) -
        $signed({previous_position[31],previous_position});
    wire [32:0] abs_arm_delta = arm_delta[32] ? -arm_delta : arm_delta;
    wire candidate = near_upright && velocity_valid && abs_delta <= CAPTURE_SPEED_ADC*2 &&
        (!(LAB_ENABLE!=0 && lab_config_words[9*16+5]) || catch_observation_valid) &&
        abs_arm_delta <= CAPTURE_ARM_SPEED_COUNTS;
    wire signed [32:0] displacement = $signed({position[31],position}) - $signed({origin[31],origin});
    wire [32:0] abs_displacement = displacement[32] ? -displacement : displacement;
    wire [32:0] abs_arm_velocity = arm_velocity_counts[32] ? -arm_velocity_counts : arm_velocity_counts;
    wire signed [63:0] predicted_displacement = $signed(displacement) +
        $signed({{31{arm_velocity_counts[32]}},arm_velocity_counts}) * PREDICT_SAMPLES;
    wire [63:0] abs_prediction = predicted_displacement[63] ? -predicted_displacement : predicted_displacement;
    wire moving_outward = displacement==0 || displacement[32]==arm_velocity_counts[32];
    wire return_needed = abs_displacement>=SOFT_TRAVEL_COUNTS ||
        (PREDICT_SAMPLES>0 && moving_outward && abs_prediction>=SOFT_TRAVEL_COUNTS);
    wire overspeed = MAX_ARM_SPEED_COUNTS>0 && abs_arm_velocity>=MAX_ARM_SPEED_COUNTS;
    wire brake_needed = overspeed || (braking && abs_arm_velocity>RESUME_ARM_SPEED_COUNTS);
    wire return_settling = returning && abs_displacement<=RETURN_WINDOW_COUNTS;
    wire [15:0] brake_amplitude = BRAKE_COMMAND>MAX_COMMAND ? MAX_COMMAND : BRAKE_COMMAND;
    wire brake_positive = arm_velocity_counts[32] ? (RETURN_POLARITY==0) : (RETURN_POLARITY!=0);
    assign running = state==SWING || state==PRECAPTURE || state==BALANCE;
    assign swinging = state==SWING || state==PRECAPTURE;
    assign balancing = state==BALANCE;
    assign fault = state==FAULT;
    assign emergency = state==ESTOP;
    assign motion_safe = !running || ENABLE_HARD_TRAVEL_STOP == 0 ||
        abs_displacement < HARD_TRAVEL_COUNTS;
    assign swing_command = swinging && sensor_fresh && !stop && !estop &&
        !encoder_illegal && motion_safe && (!relay_mode || catch_observation_valid) ?
        (reference_mode ? reference_command : state==PRECAPTURE ? catch_command : LAB_ENABLE!=0 ? lab_command : IMPULSE_ENABLE!=0 ? impulse_command :
        TRAJECTORY_ENABLE!=0 ? trajectory_command : drive_hold) : 16'sd0;

    // Nearest 11.25-degree cosine LUT, angle zero is upright.
    function signed [15:0] cosine_q8;
        input [31:0] distance;
        reg [4:0] bin;
        integer j;
        begin
            // Constant comparisons, not a wide combinational hardware divider.
            bin=0;
            for(j=0;j<16;j=j+1)
                if(distance*32>=HALF_X2*(2*j+1)) bin=j+1;
            case(bin)
                0:cosine_q8=16'sd256; 1:cosine_q8=16'sd251; 2:cosine_q8=16'sd237;
                3:cosine_q8=16'sd213; 4:cosine_q8=16'sd181; 5:cosine_q8=16'sd142;
                6:cosine_q8=16'sd98; 7:cosine_q8=16'sd50; 8:cosine_q8=16'sd0;
                9:cosine_q8=-16'sd50; 10:cosine_q8=-16'sd98; 11:cosine_q8=-16'sd142;
                12:cosine_q8=-16'sd181; 13:cosine_q8=-16'sd213; 14:cosine_q8=-16'sd237;
                15:cosine_q8=-16'sd251; default:cosine_q8=-16'sd256;
            endcase
        end
    endfunction
    wire signed [15:0] cosine = cosine_q8(abs_error);
    wire [15:0] speed_abs = velocity_x2[15] ? -velocity_x2 : velocity_x2;
    wire [31:0] speed_square = speed_abs * speed_abs;
    wire [63:0] kinetic_scaled = speed_square * KINETIC_SCALE_Q16;
    // E/(mgl) = cos(theta) + 2*(velocity / bottom-reference-velocity)^2.
    // Saturate impossible/high kinetic estimates before multiplication.
    wire [31:0] kinetic_q8 = speed_abs >= BOTTOM_X2*4 ? 32'd8192 :
        kinetic_scaled[47:16];
    wire energy_quiet = energy_error_q8 <= 32 && energy_error_q8 >= -32;
    wire pump_positive = velocity_x2[15] ^ cosine[15] ^ energy_error_q8[31] ^ (DRIVE_POLARITY != 0);
    wire return_positive = displacement[32] ? (RETURN_POLARITY==0) : (RETURN_POLARITY!=0);
    wire [31:0] ramp_sum = ramp_fraction + RAMP_REMAINDER;
    wire [31:0] ramp_amplitude = RAMP_TICKS<=0 || elapsed>=RAMP_TICKS ? MAX_COMMAND : ramp_level;
    wire signed [15:0] amplitude = ramp_amplitude > MAX_COMMAND ? MAX_COMMAND : ramp_amplitude[15:0];
    wire signed [15:0] trajectory_command;
    wire signed [15:0] impulse_command;
    wire signed [15:0] lab_command;
    wire [3:0] lab_action;wire lab_done;
    wire relay_mode=REFERENCE_ONLY==0 && LAB_ENABLE!=0 && lab_config_words[9*16+5];
    wire catch_enabled=relay_mode && lab_config_words[9*16+6] && capture_enabled;
    wire catch_entry=catch_enabled && catch_armed && velocity_valid && catch_observation_valid &&
        abs_error<=PRECAPTURE_WINDOW_ADC*2 && abs_delta>=4 && abs_delta<=80 &&
        wrapped_error[31]!=wrapped_delta[31];
    wire energy_mode=REFERENCE_ONLY==0 && LAB_ENABLE!=0 && !relay_mode && lab_config_words[9*16+4];
    wire signed [15:0] pulse_command,energy_command,relay_command;
    wire [3:0] pulse_action,pulse_cut,energy_action,energy_cut,relay_action,relay_cut;
    wire pulse_done,energy_done,relay_done;
    wire [15:0] relay_step,relay_last,relay_index;
    wire signed [15:0] relay_pole_delta;
    assign lab_relay_delta=reference_mode ? reference_speed : state==PRECAPTURE ? catch_pole_delta : relay_pole_delta;
    wire [15:0] pulse_step,pulse_last,pulse_index,energy_step,energy_last,energy_index;
    assign lab_command=relay_mode ? relay_command : energy_mode ? energy_command : pulse_command;
    assign lab_action=relay_mode ? relay_action : energy_mode ? energy_action : pulse_action;
    assign lab_done=REFERENCE_ONLY!=0 ? 1'b0 : relay_mode ? relay_done : energy_mode ? energy_done : pulse_done;
    assign lab_cut_reason=REFERENCE_ONLY!=0 ? 4'd0 : relay_mode ? relay_cut : energy_mode ? energy_cut : pulse_cut;
    assign lab_step_ms=state==PRECAPTURE ? catch_age[16:1] : relay_mode ? relay_step : energy_mode ? energy_step : pulse_step;
    assign lab_last_accel_ms=relay_mode ? relay_last : energy_mode ? energy_last : pulse_last;
    assign lab_pulse_index=relay_mode ? relay_index : energy_mode ? energy_index : pulse_index;
    // Diagnostics only. The actual BALANCE capture conditions stay unchanged.
    reg [3:0] capture_diag_hold;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)capture_diag_hold<=0;
        else if(!swinging || !relay_mode)capture_diag_hold<=0;
        else if(sample_tick)capture_diag_hold<=!near_upright || !capture_enabled ? 4'd0 :
            !velocity_valid ? 4'd11 : abs_delta>CAPTURE_SPEED_ADC*2 ? 4'd12 :
            abs_arm_delta>CAPTURE_ARM_SPEED_COUNTS ? 4'd13 : 4'd14;
    end
    assign lab_capture_diag=reference_mode ? (balancing ? 4'd15 : reference_diagnostic) : !relay_mode ? 4'd0 : balancing ? 4'd15 :
        state==PRECAPTURE && !catch_observation_valid ? 4'd6 :
        state==PRECAPTURE && catch_boundary ? 4'd10 : capture_diag_hold;
    wire [3:0] trajectory_action,impulse_action;
    wire signed [7:0] trajectory_target;
    assign swing_action=reference_mode ? reference_action : state==PRECAPTURE ? (catch_boundary ? 4'd7 : 4'd8) :
        LAB_ENABLE!=0 ? lab_action : IMPULSE_ENABLE!=0 ? impulse_action : trajectory_action;
    assign swing_target_offset=LAB_ENABLE!=0 || IMPULSE_ENABLE!=0 ? 8'sd0 : trajectory_target;
    // The R5 direction requests ACCELERATION, not an endpoint position.
    // Reject saturation/dead-zone derivatives instead of interpreting them as energy.
    wire impulse_angle_valid=angle_x2>8 && angle_x2<2038 && abs_delta<=160;
    swing_lab_controller #(.RETURN_POLARITY(RETURN_POLARITY),.DRIVE_POLARITY(DRIVE_POLARITY)) lab(
        .clk(clk),.rst_n(rst_n),.enable(REFERENCE_ONLY==0 && swinging && LAB_ENABLE!=0 && !energy_mode && !relay_mode && !reference_mode && sensor_fresh &&
        !stop && !estop && !encoder_illegal && motion_safe),.update(sample_tick),
        .config_words(lab_config_words),.displacement(displacement),.arm_velocity(arm_delta),
        .phase_positive(wrapped_delta[31] ^ cosine[15] ^ (DRIVE_POLARITY!=0)),
        .phase_valid(velocity_valid && abs_delta>=4 && impulse_angle_valid),
        .angle_valid(impulse_angle_valid),.capture_zone(near_upright),.command(pulse_command),
        .action(pulse_action),.cut_reason(pulse_cut),.step_ms(pulse_step),
        .last_accel_ms(pulse_last),.pulse_index(pulse_index),.done(pulse_done));
    swing_energy_controller #(.RETURN_POLARITY(RETURN_POLARITY),.DRIVE_POLARITY(DRIVE_POLARITY)) energy_lab(
        .clk(clk),.rst_n(rst_n),.enable(swinging && energy_mode && sensor_fresh &&
        !stop && !estop && !encoder_illegal && motion_safe),.update(sample_tick),
        .config_words(lab_config_words),.displacement(displacement),.arm_velocity(arm_delta),
        .energy_error_q8(energy_error_q8),.cosine(cosine),
        .phase_positive(wrapped_delta[31] ^ cosine[15] ^ (DRIVE_POLARITY!=0)),
        .phase_valid(velocity_valid && abs_delta>=4 && impulse_angle_valid),
        .angle_valid(impulse_angle_valid),.capture_zone(near_upright),.command(energy_command),
        .action(energy_action),.cut_reason(energy_cut),.step_ms(energy_step),
        .last_accel_ms(energy_last),.pulse_index(energy_index),.done(energy_done));
    swing_velocity_relay #(.ANGLE_PERIOD_ADC(ANGLE_PERIOD_ADC),.RETURN_POLARITY(RETURN_POLARITY),.DRIVE_POLARITY(DRIVE_POLARITY)) velocity_relay(
        .clk(clk),.rst_n(rst_n),.enable(state==SWING && relay_mode && sensor_fresh &&
        !stop && !estop && !encoder_illegal && motion_safe),.update(relay_tick),
        .config_words(lab_config_words),.angle_x2(angle_x2),.displacement(displacement),
        .arm_velocity(sample_tick ? arm_delta : arm_velocity_counts),.capture_zone(near_upright),
        .command(relay_command),.delta_x2(relay_pole_delta),.action(relay_action),.cut_reason(relay_cut),
        .step_ms(relay_step),.last_accel_ms(relay_last),.pulse_index(relay_index),.done(relay_done));
    swing_precapture #(.UPRIGHT_ADC(UPRIGHT_ADC),.BALANCE_POLARITY(BALANCE_POLARITY),
        .RETURN_POLARITY(RETURN_POLARITY),.HARD_TRAVEL(HARD_TRAVEL_COUNTS)) precapture(
        .clk(clk),.rst_n(rst_n),.enable(swinging && relay_mode && sensor_fresh && !stop && !estop &&
        !encoder_illegal && motion_safe),.active(state==PRECAPTURE),.update(relay_tick),
        .angle_x2(angle_x2),.displacement(displacement),
        .arm_velocity(sample_tick ? arm_delta : arm_velocity_counts),
        .command_cap(lab_config_words[4*16+:16]),.command(catch_command),
        .observation_valid(catch_observation_valid),.boundary_limited(catch_boundary),.pole_delta_x2(catch_pole_delta));
    swing_reference_bridge reference_bridge(.clk(clk),.rst_n(rst_n),
        .enable(state==SWING && reference_mode && sensor_fresh && !stop && !estop && !encoder_illegal && motion_safe),
        .update_ms(reference_update_ms),.angle_x2(angle_x2),.upright_adc(live_upright),
        .relative_position(displacement[31:0]),.config_words(lab_config_words),
        .qualified(reference_qualified),.command(reference_command),.pole_speed_q8(reference_speed),
        .capture_request(reference_capture),.fault_kind(reference_fault),
        .action(reference_action),.diagnostic(reference_diagnostic));
    impulse_swing_controller #(.MAX_COMMAND(IMPULSE_COMMAND),.KICK_DISTANCE(IMPULSE_DISTANCE),
        .KICK_SAMPLES(IMPULSE_SAMPLES),.KICK_SPEED(IMPULSE_SPEED),
        .EMERGENCY_SPEED(IMPULSE_EMERGENCY_SPEED),.SOFT_TRAVEL(SOFT_TRAVEL_COUNTS),
        .PREDICT_SAMPLES(PREDICT_SAMPLES),.RETURN_WINDOW(RETURN_WINDOW_COUNTS),
        .RETURN_POLARITY(RETURN_POLARITY),.DRIVE_POLARITY(DRIVE_POLARITY)) impulse(
        .clk(clk),.rst_n(rst_n),.enable(swinging && IMPULSE_ENABLE!=0 && sensor_fresh &&
        !stop && !estop && !encoder_illegal && motion_safe),.update(sample_tick),
        .displacement(displacement),.arm_velocity(arm_delta),
        .phase_positive(wrapped_delta[31] ^ cosine[15] ^ (DRIVE_POLARITY!=0)),
        .phase_valid(velocity_valid && abs_delta>=4 && impulse_angle_valid),
        .angle_valid(impulse_angle_valid),.capture_zone(near_upright),
        .command(impulse_command),.action(impulse_action));
    wire [7:0] trajectory_stroke;
    wire phase_request_positive=wrapped_delta[31] ^ cosine[15] ^ energy_error_q8[31] ^ (DRIVE_POLARITY!=0);
    swing_trajectory_controller #(.START_STROKE(TRAJECTORY_START_STROKE),.MAX_STROKE(TRAJECTORY_MAX_STROKE),
        .STROKE_STEP(TRAJECTORY_STROKE_STEP),.MAX_SPEED(TRAJECTORY_MAX_SPEED),
        .EMERGENCY_SPEED(TRAJECTORY_EMERGENCY_SPEED),.POSITION_GAIN_Q8(TRAJECTORY_POSITION_GAIN_Q8),
        .VELOCITY_GAIN(TRAJECTORY_VELOCITY_GAIN),.FEEDFORWARD_GAIN(TRAJECTORY_FEEDFORWARD_GAIN),
        .ACCEL_STEP_Q8(TRAJECTORY_ACCEL_STEP_Q8),.PHASE_HOLD(TRAJECTORY_PHASE_HOLD),
        .MAX_COMMAND(MAX_COMMAND),.SOFT_TRAVEL(SOFT_TRAVEL_COUNTS),.PREDICT_SAMPLES(PREDICT_SAMPLES),
        .RETURN_WINDOW(RETURN_WINDOW_COUNTS),.SETTLE_SPEED(RESUME_ARM_SPEED_COUNTS),
        .RETURN_POLARITY(RETURN_POLARITY),.DRIVE_POLARITY(DRIVE_POLARITY)) trajectory(
        .clk(clk),.rst_n(rst_n),.enable(swinging && TRAJECTORY_ENABLE!=0 && sensor_fresh && !stop && !estop && !encoder_illegal && motion_safe),
        .update(sample_tick),.displacement(displacement),.arm_velocity(arm_delta),
        .phase_positive(phase_request_positive),.phase_valid(velocity_valid && abs_delta>=4),
        .energy_quiet(energy_quiet),.energy_low(energy_error_q8>32),.capture_zone(near_upright),
        .effort_limit(amplitude),.command(trajectory_command),.target_offset(trajectory_target),
        .action(trajectory_action),.stroke(trajectory_stroke));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            catch_age<=0;catch_armed<=1;
            state<=IDLE;origin<=0;previous_position<=0;previous_angle<=0;velocity_x2<=0;
            drive_hold<=0;velocity_count<=0;elapsed<=0;capture_count<=0;trip_count<=0;
            velocity_valid<=0;capture_balance<=0;reason<=0;energy_error_q8<=0;
            ramp_level<=START_COMMAND;ramp_fraction<=0;
            arm_velocity_counts<=0;braking<=0;returning<=0;
        end else begin
            capture_balance<=0;
            if(update) begin
                if(sample_tick) begin
                    previous_angle <= $signed({1'b0,angle_x2});
                    previous_position <= position;
                    arm_velocity_counts <= arm_delta;
                    velocity_x2<=wrapped_delta[15:0];velocity_count<=0;velocity_valid<=1;
                end else velocity_count<=velocity_count+1'b1;
                energy_error_q8<=32'sd256-$signed(cosine)-$signed(kinetic_q8);
            end
            // Raw hardware stop pins enter here and also gate the top-level pins.
            if(estop) begin state<=ESTOP;drive_hold<=0;reason<=2;end
            else if(state==FAULT || state==ESTOP) drive_hold<=0;
            else if(stop) begin state<=IDLE;drive_hold<=0;capture_count<=0;reason<=1;end
            else if(running && !sensor_fresh) begin state<=FAULT;drive_hold<=0;reason<=4;end
            else if(running && encoder_illegal) begin state<=FAULT;drive_hold<=0;reason<=7;end
            else if(running && !motion_safe) begin state<=FAULT;drive_hold<=0;reason<=6;end
            else if(state==SWING && reference_mode && reference_fault!=0)begin
                state<=FAULT;drive_hold<=0;reason<=reference_fault==1 ? 4'd11 : 4'd12;
            end
            else if(state==SWING && reference_mode && reference_capture && reference_qualified && capture_enabled)begin
                state<=BALANCE;capture_balance<=1;drive_hold<=0;trip_count<=0;capture_count<=0;
            end
            else if(state==IDLE) begin
                catch_age<=0;catch_armed<=1;
                drive_hold<=0;elapsed<=0;capture_count<=0;trip_count<=0;
                ramp_level<=START_COMMAND;ramp_fraction<=0;
                braking<=0;returning<=0;arm_velocity_counts<=0;
                if(start && sensor_fresh) begin
                    if(encoder_illegal) begin state<=FAULT;reason<=7;end
                    else begin
                        state<=SWING;origin<=position;reason<=0;
                        previous_angle<=$signed({1'b0,angle_x2});velocity_x2<=0;
                        previous_position<=position;
                        velocity_count<=0;velocity_valid<=0;energy_error_q8<=512;
                    end
                end
            end else if(swinging && update) begin
                if(state==SWING && LAB_ENABLE!=0 && lab_done)begin
                    state<=lab_cut_reason==7 || (relay_mode && lab_cut_reason==6) ? FAULT : IDLE;drive_hold<=0;
                    reason<=lab_cut_reason==7 ? 4'd9 : relay_mode && lab_cut_reason==6 ? 4'd11 : 4'd8;
                end
                else if(elapsed>=(LAB_ENABLE!=0 ? lab_config_words[11*16+:16]*2 : TIMEOUT_TICKS)-1) begin state<=FAULT;drive_hold<=0;reason<=5;end
                else begin
                    elapsed<=elapsed+1'b1;
                    if(state==SWING)begin
                        catch_age<=0;
                        if(abs_error>=(PRECAPTURE_EXIT_ADC+20)*2)catch_armed<=1;
                        if(sample_tick && catch_entry)begin state<=PRECAPTURE;catch_age<=0;end
                    end else begin
                        catch_age<=catch_age+1'b1;
                        if(!catch_enabled || !catch_observation_valid || abs_error>PRECAPTURE_EXIT_ADC*2 ||
                            catch_age>=PRECAPTURE_TIMEOUT_TICKS-1)begin
                            state<=SWING;catch_age<=0;catch_armed<=0;capture_count<=0;
                        end
                    end
                    if(overspeed) braking<=1;
                    else if(abs_arm_velocity<=RESUME_ARM_SPEED_COUNTS) braking<=0;
                    if(return_needed) returning<=1;
                    else if(return_settling && abs_arm_velocity<=RESUME_ARM_SPEED_COUNTS) returning<=0;
                    if(elapsed<RAMP_TICKS) begin
                        ramp_level<=ramp_level+RAMP_BASE+(ramp_sum>=RAMP_DIV ? 1 : 0);
                        ramp_fraction<=ramp_sum>=RAMP_DIV ? ramp_sum-RAMP_DIV : ramp_sum;
                    end
                    if(reference_mode || !near_upright || !capture_enabled) capture_count<=0;
                    else if(sample_tick) begin
                        if(!candidate) capture_count<=0;
                        else if(capture_count>=CAPTURE_CONFIRM_SAMPLES-1) begin
                            state<=BALANCE;capture_balance<=1;drive_hold<=0;trip_count<=0;
                        end else capture_count<=capture_count+1'b1;
                    end
                    // Active reverse effort is allowed ONLY in valid SWING.
                    // Stop/estop/fault still gate all outputs to zero immediately.
                    if(brake_needed || (return_settling && abs_arm_velocity>RESUME_ARM_SPEED_COUNTS))
                        drive_hold<=brake_positive ? $signed(brake_amplitude) : -$signed(brake_amplitude);
                    else if(return_settling && !return_needed) drive_hold<=0;
                    else if(return_needed || returning)
                        drive_hold<=return_positive ? amplitude : -amplitude;
                    else if(near_upright && (!velocity_valid || speed_abs<=CAPTURE_SPEED_ADC*2)) drive_hold<=0;
                    else if(elapsed<KICK_TICKS)
                        drive_hold<=DRIVE_POLARITY!=0 ? -amplitude : amplitude;
                    else if(energy_quiet) drive_hold<=0;
                    else if(speed_abs!=0) drive_hold<=pump_positive ? amplitude : -amplitude;
                    // At a turning point retain direction, not a stale smaller
                    // amplitude. After returning to rest, restart a bounded seed
                    // instead of remaining at zero until the timeout.
                    else if(drive_hold!=0) drive_hold<=drive_hold[15] ? -amplitude : amplitude;
                    else drive_hold<=DRIVE_POLARITY!=0 ? -amplitude : amplitude;
                end
            end else if(state==BALANCE && update) begin
                drive_hold<=0;
                if(abs_error<=TRIP_WINDOW_ADC*2) trip_count<=0;
                else if(trip_count>=TRIP_CONFIRM_TICKS-1) begin state<=FAULT;reason<=3;end
                else trip_count<=trip_count+1'b1;
            end
        end
    end
endmodule
