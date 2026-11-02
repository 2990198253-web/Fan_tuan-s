// Actual Gowin top. Legacy top defaults keep historical regression reproducible.
module swing_lab_top(
    input wire clk_50m,rst_n,uart_rx,
    input wire [3:0] key_sw,input wire [9:0] adc_data_in,
    input wire enc1_a,enc1_b,enc2_a,enc2_b,
    output wire adc_clk,adc_oe_n,AN1,AN2,PWMA,BN1,BN2,PWMB,uart_tx
);
    // Same verified J280 board: reference swing hard stroke 150deg ~=448 counts.
    // R11 temporarily disables only the arm-position hard travel stop.
    // Pole-angle trip, sensor faults and K2/K4 stops remain active.
    pendulum_v6_top #(.SWING_LAB_ENABLE(1),.LAB_REFERENCE_DEFAULT(1),.LAB_REFERENCE_ENABLE(1),.LAB_REFERENCE_ONLY(1),
        .SWING_HARD_TRAVEL_COUNTS(448),.ENABLE_HARD_TRAVEL_STOP(0),
        .POSITION_KP_Q8(16'sd15),
        .POSITION_SOFT_BOUND_MARGIN_COUNTS(0),.POSITION_SOFT_BOUND_GAIN_Q8(32),
        .POSITION_SOFT_CLIP_KNEE_ADC(4),
        .POSITION_DISABLE_GUARD_WITH_ZERO_GAINS(1),
        .POSITION_KI_Q16(16'sd19),.POSITION_KD_Q8(16'sd5000),
        .KP_DUTY_PER_COUNT(16'sd115),.KI_DUTY_PER_COUNT_Q8(16'sd64),
        .KD_DUTY_PER_COUNT(16'sd520)) core(
        .clk_50m(clk_50m),.rst_n(rst_n),.uart_rx(uart_rx),.key_sw(key_sw),.adc_data_in(adc_data_in),
        .enc1_a(enc1_a),.enc1_b(enc1_b),.enc2_a(enc2_a),.enc2_b(enc2_b),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.AN1(AN1),.AN2(AN2),.PWMA(PWMA),
        .BN1(BN1),.BN2(BN2),.PWMB(PWMB),.uart_tx(uart_tx));
endmodule
