`timescale 1ns/1ps

module MIPS_top_tb;

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

    // Initialize array (word addressable)
    dcache_we = 1;
    DAddr_in = 0; DData_in = 32'd42; #10;
    DAddr_in = 1; DData_in = 32'd23; #10;
    DAddr_in = 2; DData_in = 32'd16; #10;
    DAddr_in = 3; DData_in = 32'd8; #10;
    DAddr_in = 4; DData_in = 32'd156;  #10;
    dcache_we = 0;

    // Instruction memory write enable
    icache_we = 1;

    // Instruction list (insertion sort) - addresses 0 .. 22

    // 0: addi $t0, $zero, 5          # size = 5 ($t0 = reg 8)
    IAddr_in=0;  IData_in=32'b001000_00000_01000_0000000000000101; #10;

    // 1: addi $t1, $zero, 1          # i = 1 ($t1 = reg 9)
    IAddr_in=1;  IData_in=32'b001000_00000_01001_0000000000000001; #10;

    // 2: beq  $t1, $t0, end          # if i == size jump to end (target index 22)
    // offset = 22 - (2+1) = 19
    IAddr_in=2;  IData_in=32'b000100_01001_01000_0000000000010011; #10;

    // 3: sll  $t3, $t1, 0            # key address = i ($t3 = reg 11)
    IAddr_in=3;  IData_in=32'b000000_00000_01001_01011_00000_000000; #10;

    // 4: lw $t3, 0($t3)              # key = array[i]
    IAddr_in=4;  IData_in=32'b100011_01011_01011_0000000000000000; #10;

    // 5: add $t2, $t1, $zero         # j = i ($t2 = reg 10)
    IAddr_in=5;  IData_in=32'b000000_01001_00000_01010_00000_100000; #10;

    // 6: addi $t2, $t2, -1           # j = j - 1
    IAddr_in=6;  IData_in=32'b001000_01010_01010_1111111111111111; #10;

    // 7: slt $t9, $t2, $zero         # t9 = (j < 0) ? 1 : 0   (use $t9 as temp reg 25)
    IAddr_in=7;  IData_in=32'b000000_01010_00000_11001_00000_101010; #10;

    // 8: bne $t9, $zero, insert_key  # if j < 0 jump to insert_key (insert_key at index 18)
    // offset = 18 - (8+1) = 9
    IAddr_in=8;  IData_in=32'b000101_11001_00000_0000000000001001; #10;

    // 9: sll $t4, $t2, 0             # reg 12 addr = j
    IAddr_in=9;  IData_in=32'b000000_00000_01010_01100_00000_000000; #10;

    // 10: lw $t4, 0($t4)             # load array[j]
    IAddr_in=10; IData_in=32'b100011_01100_01100_0000000000000000; #10;

    // 11: slt $t5, $t3, $t4          # t5 = (key < array[j])
    IAddr_in=11; IData_in=32'b000000_01011_01100_01101_00000_101010; #10;

    // 12: beq $t5, $zero, insert_key # if key >= array[j] jump to insert_key
    // offset = 18 - (12+1) = 5
    IAddr_in=12; IData_in=32'b000100_01101_00000_0000000000000101; #10;

    // 13: sll $t6, $t2, 0            # reg 14 addr = j
    IAddr_in=13; IData_in=32'b000000_00000_01010_01110_00000_000000; #10;

    // 14: lw $t6, 0($t6)             # array[j]
    IAddr_in=14; IData_in=32'b100011_01110_01110_0000000000000000; #10;

    // 15: addi $t7, $t2, 1           # reg 15 addr = j+1
    IAddr_in=15; IData_in=32'b001000_01010_01111_0000000000000001; #10;

    // 16: sw $t6, 0($t7)             # array[j+1] = array[j]
    IAddr_in=16; IData_in=32'b101011_01111_01110_0000000000000000; #10;

    // 17: j inner_loop_start (index 6)
    IAddr_in=17; IData_in=32'b000010_00000000000000000000000110; #10;

    // 18: insert_key:
    // addi $t2, $t2, 1             # target index j+1
    IAddr_in=18; IData_in=32'b001000_01010_01010_0000000000000001; #10;

    // 19: sw $t3, 0($t2)            # array[j+1] = key
    IAddr_in=19; IData_in=32'b101011_01010_01011_0000000000000000; #10;

    // 20: addi $t1, $t1, 1          # i++
    IAddr_in=20; IData_in=32'b001000_01001_01001_0000000000000001; #10;

    // 21: j outer_loop_start (index 2)
    IAddr_in=21; IData_in=32'b000010_00000000000000000000000010; #10;

    // 22: end: nop
    IAddr_in=22; IData_in=32'b000000_00000_00000_00000_00000_000000; #10;

    icache_we = 0;
    #20;
    start = 1;
    #4000;
    $display("\nSorted array:");
    $display("Mem[0]=%d /n Mem[1]=%d /nMem[2]=%d /nMem[3]=%d /nMem[4]=%d",
            uut.datapath_inst.D_cache.regs[0], uut.datapath_inst.D_cache.regs[1],
            uut.datapath_inst.D_cache.regs[2], uut.datapath_inst.D_cache.regs[3],
            uut.datapath_inst.D_cache.regs[4]);
    $finish;
end

always @(posedge clk) begin
    $display(" $t0=%0d $t1=%0d $t2=%0d $t3=%0d $t5=%0d $t6=%0d",
        uut.datapath_inst.reg_file.regs[8],  // size
        uut.datapath_inst.reg_file.regs[9],  // i
        uut.datapath_inst.reg_file.regs[10], // j
        uut.datapath_inst.reg_file.regs[11], // key
        uut.datapath_inst.reg_file.regs[13], // comparison reg t5
        uut.datapath_inst.reg_file.regs[14]  // temp reg t6
    );
end

endmodule
