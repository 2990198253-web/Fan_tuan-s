module quadrature_counter #(
    // Compatibility default; production top explicitly selects 50 cycles.
    parameter integer FILTER_CYCLES = 1
) (
    input wire clk,
    input wire rst_n,
    input wire phase_a,
    input wire phase_b,
    output reg signed [31:0] position,
    output reg illegal_seen
);
    reg [2:0] a_sync;
    reg [2:0] b_sync;
    reg [1:0] previous;
    reg [2:0] startup_count;
    reg initialized;
    function integer count_width;
        input integer n;
        integer value;
        begin value=n-1;count_width=0;
            while(value>0)begin count_width=count_width+1;value=value>>1;end
            if(count_width<1)count_width=1;
        end
    endfunction
    localparam integer FILTER_WIDTH=count_width(FILTER_CYCLES);
    reg [FILTER_WIDTH-1:0] stable_count;
    reg [1:0] candidate_ab, filtered_ab;
    reg filter_valid;
    wire [1:0] synchronized_ab = {a_sync[2], b_sync[2]};
    wire [1:0] current = FILTER_CYCLES<=1 ? synchronized_ab : filtered_ab;
    wire input_ready = FILTER_CYCLES<=1 || filter_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 0;
            b_sync <= 0;
            previous <= 0;
            startup_count <= 0;
            initialized <= 1'b0;
            position <= 0;
            illegal_seen <= 1'b0;
            stable_count<=0;candidate_ab<=0;filtered_ab<=0;filter_valid<=0;
        end else begin
            a_sync <= {a_sync[1:0], phase_a};
            b_sync <= {b_sync[1:0], phase_b};
            // Qualify the AB pair together: reject short pulses without
            // independently delaying A/B and inventing an intermediate state.
            if(synchronized_ab!=candidate_ab)begin
                candidate_ab<=synchronized_ab;stable_count<=1;
            end else if(stable_count<FILTER_CYCLES-1)stable_count<=stable_count+1'b1;
            else begin filtered_ab<=candidate_ab;filter_valid<=1;end
            if (!initialized && input_ready) begin
                // Establish a zero reference at the actual idle AB state.
                // Reset may be released with either input high.
                if (startup_count == 4) begin
                    previous <= current;
                    initialized <= 1'b1;
                end else begin
                    startup_count <= startup_count + 1'b1;
                end
            end else if (initialized && current != previous) begin
                // Same sign convention as the proven Gowin example:
                // 00 -> 10 -> 11 -> 01 -> 00 is positive.
                case ({previous, current})
                    4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00:
                        position <= position + 1'b1;
                    4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00:
                        position <= position - 1'b1;
                    default: illegal_seen <= 1'b1;
                endcase
                previous <= current;
            end
        end
    end
endmodule
