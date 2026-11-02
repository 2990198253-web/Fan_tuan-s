module adc_capture #(
    parameter integer SAMPLE_DIV = 10000,
    parameter integer AVERAGE_LOG2 = 4
) (
    input wire clk,
    input wire rst_n,
    input wire [9:0] adc_data,
    output reg adc_clk,
    output wire adc_oe_n,
    output reg [9:0] angle_raw,
    output reg [10:0] angle_x2,
    output reg angle_valid,
    output reg angle_update
);
    localparam integer ACC_WIDTH = 10 + AVERAGE_LOG2;
    reg [31:0] sample_divider;
    localparam integer AVERAGE_POINTS = 1 << AVERAGE_LOG2;
    reg [AVERAGE_LOG2-1:0] sample_count;
    reg [AVERAGE_LOG2-1:0] write_index;
    reg [9:0] samples [0:AVERAGE_POINTS-1];
    integer i;
    reg [ACC_WIDTH-1:0] sample_sum;
    wire [ACC_WIDTH+1:0] widened_sum = {2'b0, sample_sum} + adc_data - samples[write_index];
    wire [ACC_WIDTH:0] completed_sum = widened_sum[ACC_WIDTH:0];
    wire [31:0] rounded_sum = completed_sum + (1 << (AVERAGE_LOG2 - 1));
    wire [31:0] half_rounded_sum = completed_sum +
        ((AVERAGE_LOG2 > 1) ? (1 << (AVERAGE_LOG2 - 2)) : 0);

    assign adc_oe_n = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_clk <= 1'b0;
            sample_divider <= 0;
            sample_count <= 0;
            write_index <= 0;
            for (i = 0; i < AVERAGE_POINTS; i = i + 1) samples[i] <= 0;
            sample_sum <= 0;
            angle_raw <= 0;
            angle_x2 <= 0;
            angle_valid <= 1'b0;
            angle_update <= 1'b0;
        end else begin
            angle_update <= 1'b0;
            // 50 MHz / 2 = 25 MHz, matching the board-proven example.
            adc_clk <= ~adc_clk;
            if (sample_divider == SAMPLE_DIV - 1)
                sample_divider <= 0;
            else
                sample_divider <= sample_divider + 1'b1;

            // SAMPLE_DIV is even, so this captures just before the next
            // ADC rising edge, after the previous conversion has settled.
            if (sample_divider == 0 && !adc_clk) begin
                samples[write_index] <= adc_data;
                write_index <= write_index + 1'b1;
                sample_sum <= completed_sum[ACC_WIDTH-1:0];
                // Sliding average: after warm-up, update at every sample instead
                // of waiting for a complete new block. Round to nearest ADC count.
                if (angle_valid || sample_count == AVERAGE_POINTS - 1) begin
                    angle_raw <= rounded_sum[AVERAGE_LOG2+9:AVERAGE_LOG2];
                    // Preserve one fractional ADC bit before whole-count rounding.
                    // 1527 means 763.5; the physical converter remains 10-bit.
                    angle_x2 <= half_rounded_sum[AVERAGE_LOG2+9:AVERAGE_LOG2-1];
                    angle_valid <= 1'b1;
                    angle_update <= 1'b1;
                end else begin
                    sample_count <= sample_count + 1'b1;
                end
            end
        end
    end
endmodule
