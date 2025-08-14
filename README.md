## Overview
In this project I have designed **MIPS 32-bit processor architecture** and simulated at low level how Sorting Algorithm, Factorial Finding, Maximum Of An Array is executed at low level checking the corner cases after extensively analyzing them. Designed via Verilog Programming each and every detail. This project implements a fully functional MIPS 32-bit processor with advanced data dependency detection and correction logic. The processor features a complete 5-stage pipeline with sophisticated hazard detection and resolution mechanisms, making it capable of executing MIPS assembly programs efficiently while maintaining correctness in the presence of data dependencies.

### Key Highlights
- **Complete MIPS 32-bit ISA implementation**
- **Not only can run sorting algorithm kind of complex programs but also can run by microprogramming using problem solving skills**
- **5-stage pipeline with hazard detection**
- **Data forwarding and stalling mechanisms**
- **Branch prediction and resolution**
- **Comprehensive instruction set support**
- **Modular and extensible design**
- **Extensive testing suite**

### Core Components
1. **Instruction Fetch Unit (IF)**
2. **Instruction Decode Unit (ID)**
3. **Execution Unit (EX)**
4. **Memory Access Unit (MEM)**
5. **Write Back Unit (WB)**

## Features

### Instruction Support
- **R-Type Instructions**: ADD, SUB, AND, OR, XOR, NOR, SLT, SLL, SRL, SRA, JR
- **I-Type Instructions**: ADDI, ANDI, ORI, XORI, SLTI, LW, SW, BEQ, BNE, LUI
- **J-Type Instructions**: J, JAL

### Advanced Features
- **Data Forwarding**: Eliminates many pipeline stalls by forwarding results directly
- **Hazard Detection**: Comprehensive detection of RAW, WAR, and WAW hazards
- **Branch Prediction**: Basic branch prediction to reduce branch penalties
- **Load-Use Hazard Handling**: Automatic stall insertion for load-use dependencies
- **Exception Handling**: Basic exception and interrupt support

### Performance Optimizations
- **Forwarding Paths**: EX-to-EX, MEM-to-EX, and WB-to-EX forwarding
- **Branch Resolution**: Early branch resolution in ID stage
- **Speculation**: Basic speculative execution for branches
- **Pipeline Interlocking**: Automatic pipeline control for hazards

### Resolution Strategies

#### Data Forwarding
- **EX Forwarding**: Forward ALU result to next instruction's ALU input
- **MEM Forwarding**: Forward memory data to ALU input
- **WB Forwarding**: Forward write-back data to ALU input

#### Pipeline Stalling
- **Load-Use Stalls**: Mandatory stalls for load instructions followed by dependent instructions
- **Branch Stalls**: Stalls until branch outcome is determined
- **Resource Conflicts**: Stalls for structural hazards

### Design Methodology
- **Modular Design**: Each component is independently testable
- **Parameterizable**: Key parameters (register width, memory size) are configurable
- **Scalable**: Easy to extend with additional instructions or features
- **Verified**: Comprehensive test suite with corner cases

### Coding Standards
- **Verilog**: IEEE 1364-2005 compliant
- **Naming**: Consistent module and signal naming conventions
- **Documentation**: Comprehensive inline comments
- **Synthesis**: Synthesizable code for FPGA implementation

## Usage Examples

### Example 1: Basic Arithmetic
```assembly
# Program: basic_arithmetic.asm
main:
    # Load immediate values
    addi $t0, $zero, 15        # $t0 = 15
    addi $t1, $zero, 25        # $t1 = 25
    
    # Arithmetic operations
    add  $t2, $t0, $t1         # $t2 = $t0 + $t1 = 40
    sub  $t3, $t1, $t0         # $t3 = $t1 - $t0 = 10
    
    # Logical operations
    and  $t4, $t0, $t1         # $t4 = $t0 & $t1
    or   $t5, $t0, $t1         # $t5 = $t0 | $t1
    
    # Comparison
    slt  $t6, $t0, $t1         # $t6 = ($t0 < $t1) ? 1 : 0
```

### Example 2: Data Hazard Demonstration
```assembly
# Program: data_hazards.asm
hazard_demo:
    # RAW hazard example
    add  $t0, $t1, $t2         # Instruction 1: Write $t0
    sub  $t3, $t0, $t4         # Instruction 2: Read $t0 (hazard!)
    # Forwarding resolves this automatically
    
    # Load-use hazard
    lw   $t5, 0($sp)           # Load from memory
    add  $t6, $t5, $t7         # Use loaded value (stall needed)
    
    # Multiple dependencies
    add  $s0, $s1, $s2         # Write $s0
    sub  $s3, $s0, $s4         # Read $s0 (forwarded)
    or   $s5, $s0, $s6         # Read $s0 (forwarded)
    and  $s7, $s0, $s1         # Read $s0 (forwarded)
```

### Example 3: Branch Instructions
```assembly
# Program: branch_test.asm
branch_test:
    addi $t0, $zero, 10        # $t0 = 10
    addi $t1, $zero, 20        # $t1 = 20
    
loop:
    beq  $t0, $t1, end         # Branch if $t0 == $t1
    addi $t0, $t0, 1           # Increment $t0
    j    loop                  # Jump back to loop
    
end:
    add  $t2, $t0, $t1         # $t2 = final sum
```

