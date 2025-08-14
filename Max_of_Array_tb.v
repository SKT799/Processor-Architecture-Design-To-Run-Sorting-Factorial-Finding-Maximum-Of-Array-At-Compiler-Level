`timescale 1ns/1ps

module Max_of_Array_tb;

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

        // Initialize word-addressable data memory with the array [12, 7, 25, 3, 15]
        dcache_we = 1;
        DAddr_in = 0;  DData_in = 32'd923; #10;  // array[0]
        DAddr_in = 1;  DData_in = 32'd7;  #10;  // array[1]
        DAddr_in = 2;  DData_in = 32'd25; #10;  // array[2]
        DAddr_in = 3;  DData_in = 32'd3;  #10;  // array[3]
        DAddr_in = 4;  DData_in = 32'd15; #10;  // array[4]
        DAddr_in = 5;  DData_in = 32'd62; #10   // array[5]
        DAddr_in = 6;  DData_in = 32'd23; #10;  // array[6]
        DAddr_in = 7;  DData_in = 32'd34; #10;  // array[7], 
        DAddr_in = 8;  DData_in = 32'd12; #10;  // array[8], 
        DAddr_in = 9;  DData_in = 32'd34; #10;  // array[9], 
        dcache_we = 0;

        // Instruction memory initialization (max in $t0)
        icache_we = 1;

        // lw $t0, 0($zero)           # max = array[0]
        IAddr_in = 0; IData_in = 32'b100011_00000_01000_0000000000000000; #10

        // addi $t1, $zero, 1         # i = 1
        IAddr_in = 1; IData_in = 32'b001000_00000_01001_0000000000000000; #10

        // addi $t2, $zero, 10         # size = 10
        IAddr_in = 2; IData_in = 32'b001000_00000_01010_0000000000001010; #10

        // loop:
        // slt $t6, $t1, $t2          # if i < size, $t6=1 else 0
        IAddr_in = 3; IData_in = 32'b000000_01001_01010_01110_00000_101010; #10

        // beq $t6, $zero, +6         # if i >= size, jump to end
        IAddr_in = 4; IData_in = 32'b000100_01110_00000_0000000000000110; #10

        // lw $t3, 0($t1)             # $t3 = array[i]        # word addressed
        IAddr_in = 5; IData_in = 32'b100011_01001_01011_0000000000000000; #10

        // slt $t5, $t0, $t3          # if max < array[i], $t5=1
        IAddr_in = 6; IData_in = 32'b000000_01000_01011_01101_00000_101010; #10

        // beq $t5, $zero, +1         # if not less, skip update max
        IAddr_in = 7; IData_in = 32'b000100_01101_00000_0000000000000001; #10

        // add $t0, $t3, $zero        # max = array[i]
        IAddr_in = 8; IData_in = 32'b000000_01011_00000_01000_00000_100000; #10

        // skip:
        // addi $t1, $t1, 1           # i++
        IAddr_in = 9; IData_in = 32'b001000_01001_01001_0000000000000001; #10

        // j 3                        # jump to loop
        IAddr_in = 10; IData_in = 32'b000010_00000000000000000000000011; #10

        // end:
        // nop
        IAddr_in = 11; IData_in = 32'b000000_00000_00000_00000_00000_000000; #10

        icache_we = 0;

        #20;
        start = 1;

        #20000;

        $display("\nMaximum value in array is: %0d", uut.datapath_inst.reg_file.regs[8]); // $t0

        $finish;
    end

    always @(posedge clk) begin
        $display("$t0=%0d $t1=%0d $t3=%0d $t5=%0d $t6=%0d",
            uut.datapath_inst.reg_file.regs[8],  // max
            uut.datapath_inst.reg_file.regs[9],  // i
            uut.datapath_inst.reg_file.regs[11], // t3 (array[i])
            uut.datapath_inst.reg_file.regs[13], // t5 (slt result)
            uut.datapath_inst.reg_file.regs[14]  // t6 (loop cmp)
        );
    end

endmodule
