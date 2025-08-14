module MIPS_top (
    input clk,
    input rst,
    
    // External instruction loading interface
    input [31:0] IData_in,        // Instruction data to be loaded
    input [31:0] IAddr_in,        // Instruction address for loading
    input icache_we,              // Instruction cache write enable
    
    // External data loading interface (for future use)
    input [31:0] DData_in,        // Data to be loaded into D-cache
    input [31:0] DAddr_in,        // Data address for loading
    input dcache_we,              // Data cache write enable
    
    // Control interface
    input start                  // Start signal to begin processing
);

//=============================================================================
// INTERNAL SIGNALS AND STATE MANAGEMENT
//=============================================================================

// Processor state
reg processor_active;
reg start_reg;
wire processor_enable;

// Edge detection for start signal
always @(posedge clk or posedge rst) begin
    if (rst) begin
        start_reg <= 1'b0;
        processor_active <= 1'b0;
    end else begin
        start_reg <= start;
        // Start processing on rising edge of start signal
        if (start && !start_reg) begin
            processor_active <= 1'b1;
        end
        // Keep running once started (can be modified to add stop conditions)
    end
end

// Processor enable: active when start has been asserted and system is not in reset
assign processor_enable = processor_active && !rst;

//=============================================================================
// CONTROL UNIT SIGNALS
//=============================================================================

// Control signals from control unit to datapath
wire select_jumpD;
wire PC_load;
wire EN_to_pipelineReg1;
wire RegWrite;
wire RegDst;
wire [2:0] ALUOp;
wire ALUsrc;
wire Slt_select;
wire shift_or_not;
wire shift_direction;
wire MemWrite;
wire MemRead;
wire Branch_beq; // Updated to match new signal names
wire Branch_bne; // Updated to match new signal names
wire MemtoReg;

// Signals from datapath to control unit
wire [5:0] opcode;
wire [5:0] funct;

//=============================================================================
// HAZARD DETECTION UNIT SIGNALS
//=============================================================================

// Hazard control inputs from datapath
wire [4:0] RsE, RtE, RsD, RtD, Rtype_waddr;
wire RegWriteE, MemtoRegE, RegWriteM, MemtoRegM, RegWriteW;
wire [4:0] WriteRegE, WriteRegM, WriteRegW;

// Hazard control outputs to datapath
wire PCSrc;
wire StallF, StallD, FlushE, FlushD;
wire [1:0] ForwardAD, ForwardBD;
wire [1:0] ForwardAE, ForwardBE;
//=============================================================================
// MODIFIED CONTROL SIGNALS FOR START LOGIC
//=============================================================================

// Gate the control signals with processor_enable
wire gated_StallF;
wire gated_StallD;
assign gated_StallF = StallF && processor_enable;
assign gated_StallD = StallD && processor_enable;

//=============================================================================
// MIPS DATAPATH INSTANTIATION
//=============================================================================

MIPS_datapath datapath_inst (
    .clk(clk),
    .rst(rst),
    
    // External instruction loading
    .IData_in(IData_in),
    .IAddr_in(IAddr_in),
    .icache_we(icache_we),
    
    // External data loading
    .DData_in(DData_in),
    .DAddr_in(DAddr_in),
    .dcache_we(dcache_we),
    
    // Control signals from control unit
    .select_jumpD(select_jumpD),
    .RegWrite(RegWrite),
    .RegDst(RegDst),
    .ALUOp(ALUOp),
    .ALUsrc(ALUsrc),
    .Slt_select(Slt_select),
    .shift_or_not(shift_or_not),
    .shift_direction(shift_direction),
    .MemWrite(MemWrite),
    .MemRead(MemRead),

    .Branch_beq(Branch_beq),  // Updated to match new signal names
    .Branch_bne(Branch_bne), // Updated to match new signal names

    .MemtoReg(MemtoReg),
    
    // Outputs to control unit
    .opcode(opcode),
    .funct(funct),
    
    // Hazard control inputs
    .StallF(gated_StallF),
    .StallD(gated_StallD),
    .FlushE(FlushE),
    .FlushD(FlushD),
    .ForwardAD(ForwardAD),
    .ForwardBD(ForwardBD),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE),
    .PCSrc(PCSrc), // Combined branch detection
    // Hazard control outputs
    .RsE(RsE),
    .RtE(RtE),
    .RsD(RsD),
    .RtD(RtD),
    .RegWriteE(RegWriteE),
    .MemtoRegE(MemtoRegE),
    .RegWriteM(RegWriteM),
    .MemtoRegM(MemtoRegM),
    .RegWriteW(RegWriteW),
    .WriteRegE(WriteRegE),
    .WriteRegM(WriteRegM),
    .WriteRegW(WriteRegW),

    .start(start)
);

//=============================================================================
// MIPS CONTROL UNIT INSTANTIATION
//=============================================================================

MIPS_control_unit control_unit_inst (
    // Inputs from datapath
    .opcode(opcode),
    .funct(funct),
    
    // Control outputs
    .select_jumpD(select_jumpD),
    .PC_load(PC_load),
    .EN_to_pipelineReg1(EN_to_pipelineReg1),
    .RegWrite(RegWrite),
    .RegDst(RegDst),
    .ALUOp(ALUOp),
    .ALUsrc(ALUsrc),
    .Slt_select(Slt_select),
    .shift_or_not(shift_or_not),
    .shift_direction(shift_direction),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .Branch_beq(Branch_beq),  // Updated to match new signal names
    .Branch_bne(Branch_bne), // Updated to match new signal names
    .MemtoReg(MemtoReg)
);

//=============================================================================
// HAZARD DETECTION UNIT INSTANTIATION
//=============================================================================

hazard_detection_unit hazard_unit_inst (
    // Register addresses for hazard detection
    .RsD(RsD),
    .RtD(RtD),
    .RsE(RsE),
    .RtE(RtE), // RtE is same as Rtype_waddr based on datapath
    .WriteRegE(WriteRegE),
    .WriteRegM(WriteRegM),
    .WriteRegW(WriteRegW),
    
    // Control signals for hazard detection
    .RegWriteE(RegWriteE),
    .MemtoRegE(MemtoRegE),
    .RegWriteM(RegWriteM),
    .MemtoRegM(MemtoRegM),
    .RegWriteW(RegWriteW),
    .PCSrc(PCSrc), // Combined branch detection
    .BranchD(Branch_beq || Branch_bne), // Combined branch detection
    .select_jumpD(select_jumpD),
    
    // Stall and flush outputs
    .StallF(StallF),
    .StallD(StallD),
    .FlushE(FlushE),
    .FlushD(FlushD),
    
    // Forwarding control outputs
    .ForwardAD(ForwardAD),
    .ForwardBD(ForwardBD),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE)
);


endmodule

//=============================================================================
// USAGE NOTES:
//=============================================================================
/*
This top module provides the following functionality:

1. INSTRUCTION LOADING PHASE:
   - Before asserting 'start', load instructions using:
     - IData_in: 32-bit instruction data
     - IAddr_in: Address where instruction should be stored
     - icache_we: Write enable for instruction cache
   - Load all instructions sequentially before starting execution

2. DATA LOADING PHASE (Optional):
   - Load initial data into D-cache if needed using:
     - DData_in, DAddr_in, dcache_we

3. EXECUTION PHASE:
   - Assert 'start' signal to begin program execution
   - The processor will start fetching and executing instructions
   - 'processor_running' output indicates active execution

4. TYPICAL USAGE SEQUENCE:
   ```
   // Reset the system
   rst = 1; #10; rst = 0;
   
   // Load instructions
   icache_we = 1;
   IAddr_in = 0; IData_in = 32'h20010001; #10; // addi $1, $0, 1
   IAddr_in = 1; IData_in = 32'h20020002; #10; // addi $2, $0, 2
   IAddr_in = 2; IData_in = 32'h00221820; #10; // add $3, $1, $2
   icache_we = 0;
   
   // Start execution
   start = 1; #10; start = 0;
   
   // Monitor execution
   // processor_running will be high during execution
   ```

5. MODIFICATIONS FOR ENHANCED MONITORING:
   - To expose current_pc and current_instruction, modify the datapath
     module to include these as outputs, then connect them in this top module.
*/