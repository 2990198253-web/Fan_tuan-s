// Fixed 40-byte CRC-protected commands. Configuration commits atomically,
// only when the mechanism is stopped/settled. No serial START command exists.
module lab_config #(parameter integer CLOCK_HZ=50000000,ENERGY_DEFAULT=0,RELAY_DEFAULT=0,CATCH_DEFAULT=0,REFERENCE_DEFAULT=0,REFERENCE_ONLY=0)(
    input wire clk,rst_n,byte_valid,frame_error,safe_to_config,safe_to_target,safe_to_tune,
    input wire [7:0] byte_data,
    output reg [223:0] config_words,
    output reg [7:0] ack_sequence,ack_status,
    output reg [15:0] revision,crc_errors,
    output reg remote_stop,packet_seen,target_request,gain_request,gain_i_present,inner_gain_request,
    output reg signed [15:0] target_relative,
    output reg [15:0] target_speed,gain_p_q8,gain_i_q16,gain_d_q8,
    output reg [15:0] inner_p,inner_i_q8,inner_d
);
    // Fields0..13: accel_ms,brake_ms,wait_ms,accel_pwm,brake_pwm,
    // kick_distance,target_speed,soft_travel,pulses,flags,phase_confirm,
    // run_timeout_ms,return_pwm,capture_window_adc.
    localparam [223:0] LEGACY_DEFAULTS={16'd20,16'd1000,16'd15000,16'd2,16'd8,
        16'd1,16'd180,16'd8,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd120};
    localparam [223:0] ENERGY_DEFAULTS={16'd20,16'd1000,16'd15000,16'd2,16'd17,
        16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
    localparam [223:0] RELAY_DEFAULTS={16'd20,16'd1000,16'd15000,16'd2,16'd33,
        16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
    localparam [223:0] CATCH_DEFAULTS={16'd20,16'd1000,16'd15000,16'd2,16'd97,
        16'd40,16'd180,16'd16,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
    localparam [223:0] REFERENCE_DEFAULTS={16'd20,16'd1000,16'd15000,16'd2,16'd129,
        16'd40,16'd350,16'd24,16'd140,16'd2500,16'd2500,16'd200,16'd200,16'd40};
    reg [5:0] index;reg [31:0] age;
    reg [7:0] seq_hold,opcode,crc_low;
    reg [31:0] target_payload;
    reg [15:0] crc;reg [223:0] staging;
    reg reserved_bad;
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] val;reg [15:0] w;integer j;
        begin w=old^{val,8'd0};for(j=0;j<8;j=j+1)w=w[15] ? (w<<1)^16'h1021 : w<<1;crc_byte=w;end
    endfunction
    function valid_config;
        input [223:0] c;
        begin valid_config=
            c[0*16+:16]>=10 && c[0*16+:16]<=500 && c[0*16+:16]%10==0 &&
            c[1*16+:16]>=10 && c[1*16+:16]<=300 && c[1*16+:16]%10==0 &&
            c[2*16+:16]>=20 && c[2*16+:16]<=2000 && c[2*16+:16]%10==0 &&
            c[3*16+:16]>=500 && c[3*16+:16]<=4000 &&
            c[4*16+:16]>=500 && c[4*16+:16]<=4000 &&
            c[5*16+:16]>=20 && c[5*16+:16]<=180 &&
            c[6*16+:16]>=4 && c[6*16+:16]<=(c[9*16+7] ? 40 : 20) &&
            c[7*16+:16]>=80 && c[7*16+:16]<=(c[9*16+7] ? 360 : 200) &&
            c[5*16+:16]<=c[7*16+:16] &&
            c[8*16+:16]>=1 && c[8*16+:16]<=40 && c[9*16+:16]<=255 &&
            (!c[9*16+7] || (c[9*16+:16]&16'h007c)==0) &&
            (REFERENCE_ONLY==0 || (c[9*16+7] && (c[9*16+:16]&16'h007e)==0)) &&
            !(c[9*16+4] && c[9*16+5]) &&
            (!c[9*16+6] || c[9*16+5]) &&
            c[10*16+:16]>=1 && c[10*16+:16]<=5 &&
            c[11*16+:16]>=1000 && c[11*16+:16]<=15000 && c[11*16+:16]%10==0 &&
            c[12*16+:16]>=500 && c[12*16+:16]<=1800 &&
            c[13*16+:16]>=10 && c[13*16+:16]<=40;
        end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            config_words<=REFERENCE_DEFAULT!=0 ? REFERENCE_DEFAULTS : CATCH_DEFAULT!=0 ? CATCH_DEFAULTS : RELAY_DEFAULT!=0 ? RELAY_DEFAULTS : ENERGY_DEFAULT!=0 ? ENERGY_DEFAULTS : LEGACY_DEFAULTS;ack_sequence<=0;ack_status<=0;revision<=0;crc_errors<=0;
            remote_stop<=0;packet_seen<=0;target_request<=0;gain_request<=0;gain_i_present<=0;inner_gain_request<=0;
            target_relative<=0;target_speed<=0;gain_p_q8<=0;gain_i_q16<=0;gain_d_q8<=0;
            inner_p<=0;inner_i_q8<=0;inner_d<=0;
            target_payload<=0;index<=0;age<=0;seq_hold<=0;opcode<=0;crc_low<=0;crc<=16'hFFFF;staging<=0;reserved_bad<=0;
        end else begin
            remote_stop<=0;packet_seen<=0;target_request<=0;gain_request<=0;gain_i_present<=0;inner_gain_request<=0;
            if(index!=0)begin
                if(age>=CLOCK_HZ/20-1)begin index<=0;age<=0;end else age<=age+1'b1;
            end
            if(frame_error)begin index<=0;age<=0;end
            else if(byte_valid)begin
                age<=0;
                case(index)
                    0:if(byte_data==8'hC3)begin index<=1;crc<=16'hFFFF;reserved_bad<=0;end
                    1:if(byte_data==8'h3C)index<=6'd2;else index<=byte_data==8'hC3 ? 6'd1 : 6'd0;
                    2:if(byte_data==8'h61)begin crc<=crc_byte(crc,byte_data);index<=3;end else index<=0;
                    38:begin crc_low<=byte_data;index<=39;end
                    39:begin
                        index<=0;
                        if({byte_data,crc_low}!=crc)begin if(crc_errors!=65535)crc_errors<=crc_errors+1'b1;end
                        else begin
                            ack_sequence<=seq_hold;packet_seen<=1;
                            if(reserved_bad)ack_status<=4;
                            else if(opcode==3)begin remote_stop<=1;ack_status<=1;end
                            else if(opcode==2)ack_status<=1;
                            else if(opcode==4)begin
                                if(!safe_to_target)ack_status<=2;
                                else if($signed(target_payload[15:0]) < -16'sd180 ||
                                        $signed(target_payload[15:0]) > 16'sd180 ||
                                        target_payload[31:16] < 16'd1 || target_payload[31:16] > 16'd30)
                                    ack_status<=3;
                                else begin
                                    target_relative<=target_payload[15:0];
                                    target_speed<=target_payload[31:16];
                                    target_request<=1;ack_status<=1;
                                end
                            end
                            else if(opcode==5)begin
                                if(!safe_to_tune)ack_status<=2;
                                else if(target_payload[15:0]>16'd32 || target_payload[31:16]>16'd8191)
                                    ack_status<=3;
                                else begin
                                    gain_p_q8<=target_payload[15:0];
                                    gain_d_q8<=target_payload[31:16];
                                    gain_request<=1;ack_status<=1;
                                end
                            end
                            else if(opcode==6)begin
                                if(!safe_to_tune)ack_status<=2;
                                else if(target_payload[15:0]>16'd32 || target_payload[31:16]>16'd8191 ||
                                        staging[13*16+:16]>16'd64)
                                    ack_status<=3;
                                else begin
                                    gain_p_q8<=target_payload[15:0];
                                    gain_d_q8<=target_payload[31:16];
                                    gain_i_q16<=staging[13*16+:16];
                                    gain_request<=1;gain_i_present<=1;ack_status<=1;
                                end
                            end
                            else if(opcode==7)begin
                                if(!safe_to_tune)ack_status<=2;
                                else if(target_payload[15:0]>16'd1000 || target_payload[31:16]>16'd1200 ||
                                        staging[13*16+:16]>16'd128)
                                    ack_status<=3;
                                else begin
                                    inner_p<=target_payload[15:0];
                                    inner_d<=target_payload[31:16];
                                    inner_i_q8<=staging[13*16+:16];
                                    inner_gain_request<=1;ack_status<=1;
                                end
                            end
                            else if(opcode!=1)ack_status<=4;
                            else if(!safe_to_config)ack_status<=2;
                            else if(!valid_config(staging))ack_status<=3;
                            else begin config_words<=staging;revision<=revision+1'b1;ack_status<=1;end
                        end
                    end
                    default:begin
                        crc<=crc_byte(crc,byte_data);index<=index+1'b1;
                        if(index==3)seq_hold<=byte_data;
                        if(index==4)opcode<=byte_data;
                        if(index>=6 && index<=33)staging[(index-6)*8+:8]<=byte_data;
                        if(index>=34 && index<=37)target_payload[(index-34)*8+:8]<=byte_data;
                        if(index==5 && byte_data!=0)reserved_bad<=1;
                        if(index>=34 && index<=37 && opcode!=4 && opcode!=5 && opcode!=6 && opcode!=7 && byte_data!=0)reserved_bad<=1;
                    end
                endcase
            end
        end
    end
endmodule
