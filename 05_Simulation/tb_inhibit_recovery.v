`timescale 1ns/1ps
// A transient raw-key/angle inhibit is shorter than the debounce/trip latch.
// It must cancel in-flight control and old effort before drive resumes.
module tb_inhibit_recovery;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0;
    reg [3:0] keys=15;
    reg [9:0] adc=768;
    wire a,b,p;
    wire oa,ob,op;
    pendulum_v2_top #(.UPRIGHT_ADC(764),.ADC_AVERAGE_LOG2(4),
        .KEY_DEBOUNCE_CYCLES(40),.ADC_SAMPLE_DIV(20),
        .CONTROL_DIV(200),.SENSOR_TIMEOUT_CYCLES(4000),
        .PWM_PERIOD(50),.ANGLE_TRIP_CONFIRM_TICKS(15),
        .KD_DUTY_PER_COUNT(0),.KI_DUTY_PER_COUNT_Q8(0)) dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.adc_data_in(adc),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .BN1(a),.BN2(b),.PWMB(p));
    pendulum_v2_top #(.UPRIGHT_ADC(764),.ADC_AVERAGE_LOG2(4),
        .KEY_DEBOUNCE_CYCLES(40),.ADC_SAMPLE_DIV(20),
        .CONTROL_DIV(200),.SENSOR_TIMEOUT_CYCLES(4000),.POSITION_DIV(100),
        .ENABLE_POSITION_LOOP(1),.POSITION_KP_Q8(16'sd4),.MAX_TARGET_OFFSET_ADC(3),
        .PWM_PERIOD(50),.ANGLE_TRIP_CONFIRM_TICKS(15),
        .KD_DUTY_PER_COUNT(0),.KI_DUTY_PER_COUNT_Q8(0)) outer_dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.adc_data_in(adc),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .BN1(oa),.BN2(ob),.PWMB(op));
    integer n,kind;
    task cycles;input integer count;begin repeat(count)begin @(posedge clk);#1;end end endtask
    task reset_arm;
        begin
            @(negedge clk);rst_n=0;keys=15;adc=768;
            force outer_dut.encoder1=32'sd100;cycles(3);
            @(negedge clk);rst_n=1;cycles(400);
            @(negedge clk);keys[0]=0;cycles(50);
            @(negedge clk);keys[0]=1;force outer_dut.encoder1=32'sd180;cycles(500);
            if(!dut.armed||dut.motor_command==0)$fatal(1,"Recovery test arm failed");
            if(outer_dut.position_target!==100||outer_dut.position_offset_q8==0)
                $fatal(1,"Outer recovery test setup");
        end
    endtask
    task off_check;
        begin #1;if(a||b||p||oa||ob||op||dut.motor_command!==0||outer_dut.motor_command!==0)
            $fatal(1,"Immediate safety gate failed");end
    endtask
    initial begin #10000000;$fatal(1,"Inhibit recovery timeout");end
    initial begin
        for(kind=0;kind<3;kind=kind+1)begin
          for(n=0;n<7;n=n+1)begin
            reset_arm;
            // Sweep idle plus every stage of the PD/limiter pipeline.
            wait(dut.control_count==(n==0?20:200-n));@(negedge clk);
            if(kind==0)keys[1]=0;
            else if(kind==1)keys[3]=0;
            else begin force dut.adc.angle_x2=11'd2000;force outer_dut.adc.angle_x2=11'd2000;end
            off_check;cycles(5);
            if(!dut.armed||dut.fault_latched||dut.emergency_stop)$fatal(1,"Short inhibit incorrectly latched");
            if(outer_dut.position_offset_q8!==0||outer_dut.position_velocity_q8!==0||
               outer_dut.position_target!==100||outer_dut.position_count!==0)
                $fatal(1,"Outer inhibit must clear output/velocity/timer and preserve captured target");
            @(negedge clk);
            if(kind==0)keys[1]=1;
            else if(kind==1)keys[3]=1;
            else begin release dut.adc.angle_x2;release outer_dut.adc.angle_x2;end
            #1;
            if(dut.compensated_command!==0||dut.motor_command!==0||a||b||p||
               outer_dut.compensated_command!==0||outer_dut.motor_command!==0||oa||ob||op)
                $fatal(1,"Transient inhibit resumed stale PWM kind=%0d cmd=%0d",kind,dut.motor_command);
            // Keep the real ADC around 768. A new update may legitimately
            // request drive, but no old pipeline result may leak out first.
            while(!(dut.control_tick && dut.safe_active))begin
                cycles(1);if(dut.motor_command!==0||outer_dut.motor_command!==0)$fatal(1,"Effort before fresh control tick");
            end
            cycles(10);
            if(dut.motor_command<=0||!dut.armed)$fatal(1,"Fresh correction did not resume kind=%0d phase=%0d angle=%0d",kind,n,dut.angle_x2);
          end
        end
        release outer_dut.encoder1;
        $display("V2_INHIBIT_RECOVERY_PASS cases=21 inner_and_outer");$finish;
    end
endmodule
