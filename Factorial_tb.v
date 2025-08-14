`timescale 1ns/1ps

module Factorial_tb;

    reg clk = 0;
    reg rst;

    reg [31:0] IData_in;
    reg [31:0] IAddr_in;
    reg icache_we;

    reg [31:0] DData_in;
    reg [31:0] DAddr_in;
    reg dcache_we;

    reg start;

    wire processor_running;
    wire [31:0] current_pc;
    wire [31:0] current_instruction;

    MIPS_top uut (
        .clk(clk),
        .rst(rst),
        .IData_in(IData_in),
        .IAddr_in(IAddr_in),
        .icache_we(icache_we),
        .DData_in(DData_in),
        .DAddr_in(DAddr_in),
        .dcache_we(dcache_we),
        .start(start)
    );

    always #5 clk = ~clk;

    initial begin
        rst = 1;
        start = 0;
        icache_we = 0;
        dcache_we = 0;

        #10; 
        rst = 0;

        // No data memory initialization needed 

        // Write instructions into instruction memory
        icache_we = 1;

        // addi $t0, $zero, 1         # fact = 1
        IAddr_in = 0; IData_in = 32'b001000_00000_01000_0000000000000001; #10;

        // addi $t1, $zero, 1         # i = 1
        IAddr_in = 1; IData_in = 32'b001000_00000_01001_0000000000000001; #10;

        // addi $t2, $zero, 6         # n = 5 //for n! write n+1
        IAddr_in = 2; IData_in = 32'b001000_00000_01010_0000000000000110; #10;

        // loop:
        // mul $t0, $t0, $t1         # fact *= i
        IAddr_in = 3; IData_in = 32'b000000_01000_01001_01000_00000_101100; #10;

        // addi $t1, $t1, 1           # i++
        IAddr_in = 4; IData_in = 32'b001000_01001_01001_0000000000000001; #10;

        // slt $t3, $t1, $t2          # if i < n
        IAddr_in = 5; IData_in = 32'b000000_01001_01010_01011_00000_101010; #10;

        // beq $t3, $zero, 8          # if i >= n jump to end
        IAddr_in = 6; IData_in = 32'b000100_01011_00000_0000000000000010; #10;

        // j 3                        # jump to loop
        IAddr_in = 7; IData_in = 32'b000010_00000000000000000000000011; #10;

        // end:
        // nop
        IAddr_in = 8; IData_in = 32'b000000_00000_00000_00000_00000_000000; #10;

        icache_we = 0;

        #20;
        start = 1;

        #3000;

        $display("\nFactorial(5) stored in $t0 = %0d", uut.datapath_inst.reg_file.regs[8]);

        $finish;
    end

    always @(posedge clk) begin
        $display("$t0=%0d $t1=%0d $t2=%0d $t3=%0d", 
            uut.datapath_inst.reg_file.regs[8],  // $t0 fact
            uut.datapath_inst.reg_file.regs[9],  // $t1 i
            uut.datapath_inst.reg_file.regs[10], // $t2 n
            uut.datapath_inst.reg_file.regs[11]); // $t3
    end

endmodule