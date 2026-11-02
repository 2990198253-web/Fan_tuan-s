// Short unconfirmed-center hold, then bounded reduction. Direction reversal,
// reset and safety disable never wait for center qualification.
module balance_pwm_limiter #(
    parameter integer MAX_COMMAND = 2000,
    parameter integer MIN_ACTIVE_COMMAND = 600,
    parameter integer OUTPUT_RELEASE_STEP = 100,
    parameter integer START_COMMAND = 1200,
    parameter integer START_TICKS = 8,
    parameter integer SMOOTH_BALANCE_ENABLE = 0
) (
    input wire clk, rst_n, enable, update, inside_deadband, deadband_confirmed, position_needs_action,
    input wire signed [15:0] request,
    output reg signed [15:0] command
);
    reg [31:0] start_left;
    reg start_positive, previous_force_min, pending, hold_reduce;
    reg signed [15:0] desired_hold;
    wire force_min = !deadband_confirmed || position_needs_action;
    wire positive = !request[15];
    wire [15:0] magnitude = request[15] ? -request : request;
    // Near zero use the actual PD effort, not a +/-minimum-drive relay.
    // Large corrections can still use the existing startup boost. End it as
    // soon as demand falls back below the threshold, preserving D braking.
    wire boost_allowed = SMOOTH_BALANCE_ENABLE != 0 ?
        (magnitude >= MIN_ACTIVE_COMMAND) : force_min;
    wire [31:0] normal_amplitude = SMOOTH_BALANCE_ENABLE == 0 && force_min &&
        magnitude < MIN_ACTIVE_COMMAND ? MIN_ACTIVE_COMMAND : magnitude;
    wire start_now = request != 0 && boost_allowed && START_TICKS > 0 && START_COMMAND > 0 &&
        (command == 0 || !previous_force_min);
    wire continuing_start = start_left != 0 && boost_allowed && request != 0 && positive == start_positive;
    wire [31:0] boosted_amplitude = normal_amplitude < START_COMMAND ? START_COMMAND : normal_amplitude;
    wire [31:0] amplitude = request == 0 ? 0 : ((start_now || continuing_start) ? boosted_amplitude : normal_amplitude);
    wire [15:0] limited = amplitude > MAX_COMMAND ? MAX_COMMAND : amplitude[15:0];
    wire [15:0] current_magnitude = command[15] ? -command : command;
    wire [15:0] target_magnitude = desired_hold[15] ? -desired_hold : desired_hold;
    wire signed [31:0] release_up = $signed(command) + OUTPUT_RELEASE_STEP;
    wire signed [31:0] release_down = $signed(command) - OUTPUT_RELEASE_STEP;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            start_left <= 0; start_positive <= 0; previous_force_min <= 0;
            pending <= 0; hold_reduce <= 0; desired_hold <= 0; command <= 0;
        end else if(!enable) begin
            start_left <= 0; previous_force_min <= 0; pending <= 0;
            hold_reduce <= 0; desired_hold <= 0; command <= 0;
        end else if(update) begin
            previous_force_min <= boost_allowed; pending <= 1;
            hold_reduce <= SMOOTH_BALANCE_ENABLE == 0 && inside_deadband &&
                !deadband_confirmed && !position_needs_action;
            desired_hold <= positive ? $signed(limited) : -$signed(limited);
            if(start_now) begin start_left <= START_TICKS - 1; start_positive <= positive; end
            else if(continuing_start) start_left <= start_left - 1'b1;
            else start_left <= 0;
        end else if(pending) begin
            pending <= 0;
            if(command != 0 && desired_hold != 0 && desired_hold[15] != command[15]) command <= desired_hold;
            else if(hold_reduce && current_magnitude > target_magnitude) command <= command;
            else if(SMOOTH_BALANCE_ENABLE == 0 && OUTPUT_RELEASE_STEP > 0 && current_magnitude > target_magnitude &&
                    current_magnitude - target_magnitude > OUTPUT_RELEASE_STEP)
                command <= command[15] ? release_up[15:0] : release_down[15:0];
            else command <= desired_hold;
        end
    end
endmodule
