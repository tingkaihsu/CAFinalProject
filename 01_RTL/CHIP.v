//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    //TODO instructions
    // ADD
    // SUB 
    // XOR 
    // AND 
    // ADDI 
    // SLLI 
    // SLTI 
    // SRAI 
    // LW 
    // SW 
    // BEQ 
    // BGE 
    // BLT 
    // BNE 
    // MUL 
    // DIV 
    // ECALL 
    // TODO: any declaration
    //assign opcode parameters
    parameter ASXA = 7'b0110011;   //add, sub, xor, and
    parameter IMMD = 7'b0010011;   //addi, slli, slti, srai
    parameter LW = 7'b0000011;     //load word
    parameter SW = 7'b0100011;     //store word
    parameter BRNCH = 7'b1100011;  //beq, bge, blt, bne
    parameter MULDIV = 7'b0110011;    //multiplication and division
    parameter ECALL = 7'b1110011;
    //function3 code of instructions
    parameter FUNC3_ADD = 3'b000;
    parameter FUNC3_SUB = 3'b000;
    parameter FUNC3_XOR = 3'b100;
    parameter FUNC3_AND = 3'b111;
    parameter FUNC3_ADDI = 3'b000;
    parameter FUNC3_SLLI = 3'b001;
    parameter FUNC3_SLTI = 3'b010;
    parameter FUNC3_SRAI = 3'b101;
    parameter FUNC3_LW = 3'b010;
    parameter FUNC3_SW = 3'b010;
    parameter FUNC3_BEQ = 3'b000;
    parameter FUNC3_BGE = 3'b101;
    parameter FUNC3_BLT = 3'b100;
    parameter FUNC3_BNE = 3'b001;
    parameter FUNC3_MUL = 3'b000;
    parameter FUNC3_DIV = 3'b100;
    parameter FUNC3_ECALL = 3'b000;
    //function7 code of instruction
    parameter FUNC7_ADD = 7'b0000000;
    parameter FUNC7_SUB = 7'b0100000;
    parameter FUNC7_XOR = 7'b0000000;
    parameter FUNC7_AND = 7'b0000000;
    parameter FUNC7_SLLI = 7'b0000000;
    parameter FUNC7_SRAI = 7'b0100000;
    parameter FUNC7_MUL = 7'b0000001;
    parameter FUNC7_DIV = 7'b0000001;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

        //we should use wire to connect to the wires of register file
        //so we must create the register and its corresponding wire
        reg [4:0] RS1, RS2, RD;
        wire [4:0] rs1_wr, rs2_wr, rd_wr;
        //the data to Reg_file, which should be wire
        wire[BIT_W-1:0] WRITE_DATA, RS1_DATA, RS2_DATA;
        //signal after decoding to determine if the instr is load or ALU that 
        //needs to write back to reg
        wire write_to_reg;

        //multiplication declaration
        //input of module can eat reg and wire
        //but output must connect to wire
        reg mul_ready, mul_valid;
        reg [BIT_W-1:0] mul_rs1;
        reg [BIT_W-1:0] mul_rs2;
        reg mul_mode;   //determine if mul or div
        //output
        wire [2*BIT_W-1:0] mul_wdata;
        wire mul_ready_output;

        
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    // TODO: any wire assignment
    //attach the register to the wire
    assign rs1_wr = RS1;
    assign rs2_wr = RS2;
    assign rd_wr = RD;
    //

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submodules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    //the content in () should be wire
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (write_to_reg),          
        .rs1    (rs1_wr),                
        .rs2    (rs2_wr),                
        .rd     (rd_wr),                 
        .wdata  (WRITE_DATA),             
        .rdata1 (RS1_DATA),           
        .rdata2 (RS2_DATA)
    );
    MULDIV_unit muldiv_unit(
        .mul_clk(i_clk),
        .mul_rst_n(i_rst_n),
        .mul_valid(mul_valid),
        .mul_mode(mul_mode),
        .mul_rs1(mul_rs1),
        .mul_rs2(mul_rs2),
        .mul_done(mul_ready_output),
        .mul_data(mul_wdata)
    );
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= next_PC;
        end
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(
    // TODO: port declaration
    mul_clk, mul_rst_n, mul_valid, mul_mode, mul_done, mul_rs1, mul_rs2, mul_data
    );

    parameter len = 32;
    //valid means the muldiv_unit should work now
    input mul_clk, mul_rst_n, mul_valid;
    input mul_mode; //mode=0, multiplication; mode=1, division
    input [len-1:0] mul_rs1, mul_rs2;
    output [2*len-1:0] mul_data;
    //output the ready signal
    output mul_done;
    
    // Todo: HW2
    //state, which would be idle, one cycle operation
    //multi-cycle operation
    reg[1:0] state, state_nxt;
    parameter S_IDLE = 2'b00, S_ONE_CUCLE_OP = 2'b01, S_MULTI_CYCLE_OP = 2'b10;
    //the input would have "mul" as prefix
    reg [len-1:0] operand_a, operand_a_nxt;
    reg [len-1:0] operand_b, operand_b_nxt;
    reg mode, mode_nxt;
    reg [2*len-1:0] out_result; //this would finally connect to output
    reg [2*len:0] temp_result;  //this would deal with overflow prob
    //counter
    reg [4:0] cnt;
    reg ready_to_output;

    //wire assignment
    assign mul_done = ready_to_output;
    assign mul_data = out_result;

    //Always Combination
    //load input
    always @(*) begin
        if (mul_valid) begin
            operand_a_nxt = mul_rs1;
            operand_b_nxt = mul_rs2;
            mode_nxt = mul_mode;
        end
        else begin
        end
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS
    // TODO: any declaration
    reg [ADDR_W-1:0] addr;
    
    assign index = addr[21:12];
    assign tag = addr[11:2];

    always @(posedge i_clk or negedge i_rst_n) begin
        
    end
endmodule