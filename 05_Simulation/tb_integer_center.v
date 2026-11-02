module tb_integer_center;
    parameter integer old_integer_center = 763.5;
    initial begin
        $display("OLD_INTEGER_CENTER=%0d",old_integer_center);
        if(old_integer_center!==764) $fatal(1,"Unexpected integer conversion");
        $display("V2_INTEGER_CENTER_PASS");
        $finish;
    end
endmodule
