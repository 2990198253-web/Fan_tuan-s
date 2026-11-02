`timescale 1ns/1ps
module tb_lab_top;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,rx=1;reg [3:0] keys=15;
    reg [10:0] angle=450;reg signed [31:0] pos=0;
    wire tx,a,b,p;wire [7:0] observed;wire byte_valid,byte_error;
    pendulum_v6_top #(.SWING_LAB_ENABLE(1),.KEY_DEBOUNCE_CYCLES(2),
        .CONTROL_DIV(1000),.POSITION_DIV(2000),.LAB_CLOCK_HZ(1000000),.LAB_SETTLE_CYCLES(20),
        .UART_BAUD(100000),.TELEMETRY_DIV(10000),.ADC_SAMPLE_DIV(20),.ADC_AVERAGE_LOG2(3),
        .SENSOR_TIMEOUT_CYCLES(5000)) dut(
        .clk_50m(clk),.rst_n(rst),.uart_rx(rx),.key_sw(keys),.adc_data_in(10'd225),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .BN1(a),.BN2(b),.PWMB(p),.uart_tx(tx));
    lab_uart_rx #(.CLOCK_HZ(1000000),.BAUD(100000)) reader(
        .clk(clk),.rst_n(rst),.rx(tx),.data(observed),.valid(byte_valid),.frame_error(byte_error));
    integer output_file,bytes_seen=0,i;reg [7:0] frame[0:39];reg [223:0] cfg;
    always @(posedge clk)begin
        if(byte_valid)begin $fdisplay(output_file,"%02x",observed);bytes_seen=bytes_seen+1;end
        if(byte_error)$fatal(1,"Telemetry UART framing error");
    end
    function [15:0] crc_byte;input [15:0] old;input [7:0] value;reg [15:0] w;integer j;
        begin w=old^{value,8'd0};for(j=0;j<8;j=j+1)w=w[15] ? (w<<1)^16'h1021 : w<<1;crc_byte=w;end endfunction
    task cycles;input integer n;begin repeat(n)@(negedge clk);end endtask
    task send_bit;input bit_value;begin rx=bit_value;cycles(10);end endtask
    task send_byte;input [7:0] value;integer j;begin
        send_bit(0);for(j=0;j<8;j=j+1)send_bit(value[j]);send_bit(1);end endtask
    task send_command;input [7:0] seq,opcode;reg [15:0] crc;begin
        for(i=0;i<40;i=i+1)frame[i]=0;
        frame[0]=8'hC3;frame[1]=8'h3C;frame[2]=8'h61;frame[3]=seq;frame[4]=opcode;
        for(i=0;i<28;i=i+1)frame[6+i]=cfg[i*8+:8];crc=16'hFFFF;
        for(i=2;i<=37;i=i+1)crc=crc_byte(crc,frame[i]);frame[38]=crc[7:0];frame[39]=crc[15:8];
        for(i=0;i<40;i=i+1)send_byte(frame[i]);cycles(30);
    end endtask
    task press;input integer key;begin keys[key]=0;cycles(12);keys[key]=1;cycles(12);end endtask
    task reset_case;begin
        // Finish the captured UART frame before resetting this test instance.
        // Real mid-frame reset recovery is covered separately by Decoder tests.
        while(dut.lab_sender.state!=0)cycles(1);cycles(30);
        rst=0;keys=15;rx=1;pos=0;angle=450;cycles(5);rst=1;cycles(50);
        cfg=dut.lab_config_words;end endtask
    initial begin
        force dut.adc.angle_x2=angle;force dut.adc.angle_valid=1'b1;force dut.adc.angle_update=1'b1;
        force dut.encoder1=pos;
        output_file=$fopen("smooth_results/lab_uart_capture.hex","w");
        $dumpfile("smooth_results/lab_top.vcd");
        $dumpvars(0,dut.auto_state,dut.motor_command,dut.lab_revision,dut.lab_packet_seen,
            dut.lab_remote_stop,dut.lab_link_fault,dut.swing_action,dut.lab_last_accel_ms);
        reset_case();press(0);cycles(50);if(dut.armed)$fatal(1,"K1 allowed before host heartbeat");
        cfg[0+:16]=200;cfg[6*16+:16]=12;send_command(10,1);
        if(dut.lab_revision!=1||dut.lab_ack_seq!=10||dut.lab_ack_status!=1||dut.lab_config_words!=cfg||dut.armed)
            $fatal(1,"UART config commit or accidental start");
        cycles(20000);press(0);cycles(25000);
        if(!dut.swing_active||dut.motor_command<=0)$fatal(1,"LAB K1 did not select runtime controller");
        cfg[0+:16]=300;send_command(11,1);
        if(dut.lab_revision!=1||dut.lab_ack_status!=2||dut.lab_config_words[0+:16]!=200)$fatal(1,"Running config changed");
        cycles(15000);
        send_command(12,3);cycles(5);
        if(dut.armed||a||b||p||dut.motor_command!=0)$fatal(1,"Serial STOP failed");
        cycles(15000);
        cycles(40);cfg=dut.lab_config_words;send_command(13,2);press(0);cycles(25000);
        pos=240;#1;if(dut.motor_command!=0||a||b||p)$fatal(1,"LAB immediate hard travel gate");
        cycles(4);if(!dut.fault_latched||dut.first_inhibit_reason!=6)$fatal(1,"LAB travel latch");
        cycles(15000);
        reset_case();send_command(14,2);press(0);cycles(25000);keys[3]=0;#1;
        if(dut.motor_command!=0||a||b||p)$fatal(1,"LAB K4 raw gate");
        cycles(20);keys[3]=1;send_command(15,2);press(0);if(dut.armed||!dut.emergency_stop)$fatal(1,"Emergency restarted");
        reset_case();send_command(16,2);press(0);cycles(25000);
        force dut.adc.angle_update=1'b0;cycles(5100);
        if(!dut.fault_latched||dut.first_inhibit_reason!=4||dut.motor_command!=0)$fatal(1,"LAB stale ADC");
        force dut.adc.angle_update=1'b1;
        reset_case();cfg[9*16+:16]=9;send_command(18,1);press(0);cycles(25000);
        pos=15;angle=1510;cycles(65000);
        if(!dut.balance_mode||dut.position_target!=15||dut.fault_latched)$fatal(1,"Runtime auto-balance handoff/rebase");
        cycles(15000);
        keys[1]=0;#1;if(dut.motor_command!=0)$fatal(1,"LAB balance raw K2 stop");
        reset_case();cfg[0+:16]=40;cfg[6*16+:16]=16;cfg[8*16+:16]=40;cfg[9*16+:16]=17;
        send_command(19,1);cycles(15000);press(0);cycles(110000);
        if(!dut.swing_controller.energy_mode||!dut.swing_active)$fatal(1,"Energy strategy not selected by UART config");
        keys[1]=0;cycles(20);keys[1]=1;cycles(15000);
        reset_case();cfg[0+:16]=40;cfg[6*16+:16]=16;cfg[8*16+:16]=40;cfg[9*16+:16]=33;
        send_command(20,1);cycles(15000);press(0);cycles(100000);
        if(!dut.swing_controller.relay_mode||dut.swing_controller.energy_mode)$fatal(1,"Relay UART select");
        // 5ms producer cadence; monitor transmits every 10ms.
        repeat(8)begin angle=angle+4;cycles(10000);end
        if(dut.lab_relay_delta!=4||dut.motor_command!=2500)$fatal(1,"Relay signed delta or command mux");
        repeat(8)begin angle=angle-4;cycles(10000);end
        if(dut.lab_relay_delta!=-4||dut.motor_command!=-2500)$fatal(1,"Relay negative direction not transmitted");
        angle=1472;cycles(20000);angle=1484;cycles(20000);angle=1496;cycles(20000);angle=1508;cycles(20000);
        if(dut.balance_mode||dut.lab_capture_diag!=12)$fatal(1,"Fast pole should not capture");
        cycles(80000);if(!dut.balance_mode||dut.lab_capture_diag!=15)$fatal(1,"Relay existing balance handoff");
        cycles(15000);keys[1]=0;cycles(20);keys[1]=1;
        reset_case();cfg[0+:16]=40;cfg[6*16+:16]=16;cfg[8*16+:16]=40;cfg[9*16+:16]=97;
        send_command(21,1);cycles(15000);press(0);cycles(40000);
        repeat(45)begin angle=angle+20;cycles(10000);end
        if(dut.auto_state!=5||!dut.swing_active||!dut.armed)$fatal(1,"R1 top precapture not running");
        cycles(15000);
        repeat(6)begin angle=angle+20;cycles(10000);end
        angle=1490;cycles(80000);
        if(!dut.balance_mode||dut.lab_capture_diag!=15||dut.fault_latched)$fatal(1,"R1 top handoff to original balance");
        cycles(15000);keys[1]=0;cycles(20);keys[1]=1;
        reset_case();cfg[0+:16]=500;cfg[8*16+:16]=40;send_command(17,1);press(0);cycles(1020000);
        if(dut.armed||dut.motor_command!=0||!dut.lab_link_fault)$fatal(1,"LAB heartbeat lost did not stop");
        if(bytes_seen<64)$fatal(1,"No telemetry output");
        while(dut.lab_sender.state!=0)cycles(1);cycles(30);$fclose(output_file);
        $display("SWING_LAB_TOP_PASS UART_config atomic_busy_reject readback no_auto_start K1 serial_stop hard_travel K4 sensor heartbeat");$finish;
    end
    initial begin #100000000;$fatal(1,"Lab top timeout");end
endmodule
