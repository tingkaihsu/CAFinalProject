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
    parameter AUIPC = 7'b0010111;
    parameter JAL = 7'b1101111;
    parameter JALR = 7'b1100111;
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
    parameter FUNC3_JALR = 3'b000;
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

    parameter S_IDLE = 0, S_ONE_CYCLE_EXEC = 1, S_MULTI_CYCLE_EXEC = 2;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire dmem_stall; //data mem stall, which should be connected to the input
        reg [BIT_W-1:0] stall_counter; //count the stall
        // reg [BIT_W-1:0] mul_stall_counter;

        reg imem_cen;   //instruction mem enable
        reg dmem_cen;   //data mem enable 
        reg dmem_wen;   //data mem write enable
        //we found out enable signal should be sequential
        reg dmem_wen_nxt;
        reg dmem_cen_nxt;
        reg finish;     //finish signal, ready to output
        reg [BIT_W-1:0] dmem_wdata, dmem_rdata, dmem_addr;
        //memory write data, memory read data, memory address

        //decoded instruction
        reg [BIT_W-1:0] instr;
        reg [6:0] opcode;
        reg [6:0] FUNC7;
        reg [2:0] FUNC3;
        //use wire to in/output of register file
        //must create registers and its corresponding wires
        reg [4:0] RS1, RS2, RD;
        reg [BIT_W-1:0] temp_result;
        reg [BIT_W-1:0] immd;
        wire [4:0] rs1_wr, rs2_wr, rd_wr;//to Reg_file, which is wire
        wire [BIT_W-1:0] RS1_DATA, RS2_DATA;//to Reg_file, which is wire
        reg [BIT_W-1:0] WRITE_DATA;
        reg write_to_reg;//needs to write back to reg like ALU and LW, 0 for no, 1 for yes

        //MULDIV-unit declaration
        //input of module can eat reg and wire
        //but output must connect to wire
        reg mul_valid;
        reg [BIT_W-1:0] mul_rs1;
        reg [BIT_W-1:0] mul_rs2;
        reg [1:0] mul_mode;   //determine if mul or div
        //output
        wire [2*BIT_W-1:0] mul_wdata;
        wire mul_ready_output;
        //FSM
        reg [1:0] state, state_nxt;
        
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    // TODO: any wire assignment
    //attach the register to the wire
    assign rs1_wr = RS1;
    assign rs2_wr = RS2;
    assign rd_wr = RD;
    //attach output wire of CHIP to reg
    assign o_finish = finish;
    assign o_IMEM_addr = PC;//output the processing center of instr address
    assign o_IMEM_cen = imem_cen;//tell instr mem to operate
    assign o_DMEM_cen = dmem_cen;//tell memory to operate
    assign o_DMEM_wen = dmem_wen;//tell memory to write, for store
    assign o_DMEM_addr = dmem_addr;//data memory address
    assign o_DMEM_wdata = dmem_wdata;//write data to data memory
    //attach input 
    // assign dmem_stall = i_DMEM_stall;//input -> wire so that we know when to stall
    
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submodules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    //the content in () should be wire
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (write_to_reg),//write back to reg        
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
        if (!i_rst_n) begin//reset everything
            PC <= 32'h00010000; // Do not modify this value!!!
            dmem_cen <= 0;   //reset enable signal
            dmem_wen <= 0;   //reset enable signal
            state <= S_IDLE;
        end
        else begin
            PC <= next_PC;
            dmem_cen <= dmem_cen_nxt;
            dmem_wen <= dmem_wen_nxt;
            state <= state_nxt;
        end
    end

    //FSM
    always @(*) begin
        case (state)
            S_IDLE: begin
                if (i_DMEM_stall || stall_counter > 1) state_nxt = S_IDLE;//keep stall
                else state_nxt = ({opcode, FUNC3, FUNC7} == {MULDIV, FUNC3_MUL, FUNC7_MUL})? S_MULTI_CYCLE_EXEC : S_ONE_CYCLE_EXEC;
                //determine mul or not
            end
            S_ONE_CYCLE_EXEC: begin
                if (i_DMEM_stall) state_nxt = S_IDLE;//stall
                else state_nxt = ({opcode, FUNC3, FUNC7} == {MULDIV, FUNC3_MUL, FUNC7_MUL})? S_MULTI_CYCLE_EXEC : S_ONE_CYCLE_EXEC;
            end
            S_MULTI_CYCLE_EXEC: begin
                if (mul_ready_output == 0) state_nxt = state;//not ready to output multiplication
                else state_nxt = S_ONE_CYCLE_EXEC;
            end
            default: state_nxt = state;
        endcase
    end

    //stall counter counts when LW or SW
    always @(posedge i_clk) begin
        if ( ((opcode == LW) || (opcode == SW)) && i_DMEM_stall == 1) stall_counter = stall_counter + 1;
        else stall_counter = 0;//zero when no store or load
    end

    // always @(posedge i_clk) begin
    //     if ((opcode == MULDIV) && ({FUNC3, FUNC7} == {FUNC3_MUL, FUNC7_MUL}) && mul_valid) mul_stall_counter = mul_stall_counter + 1;
    //     else mul_stall_counter = 0;
    // end

    // always @(posedge i_clk) begin
    //     if ((opcode == MULDIV) && ({FUNC3, FUNC7} == {FUNC3_MUL, FUNC7_MUL})) mul_valid = 1;
    //     else mul_valid = 0;
    // end

    always @(*) begin
        //set the instruction-memory-access-enable to 1 if there is no imem
        //no access instr enable when stall
        // imem_cen = (i_DMEM_stall)? 0:1;
        // next_PC = (i_DMEM_stall)? PC:PC+4;
        imem_cen = 1;
        instr = i_IMEM_data; //read the instruction
        if (i_DMEM_stall) begin
            //this is made for avoiding latch
            next_PC = PC;
            finish = 0;
            write_to_reg = 0;
            //mul div
            mul_valid = 0;
            mul_mode = 0;
            mul_rs1 = 0;
            mul_rs2 = 0;
            //decoding
            opcode = instr[6:0];
            FUNC3 = instr[14:12];
            FUNC7 = instr[BIT_W-1:25];
            RS1 = instr[19:15];
            RS2 = instr[24:20];
            RD = instr[11:7];
            WRITE_DATA = 0;
            //data memory
            dmem_wdata = RS2_DATA;
            dmem_rdata = 0;
            dmem_cen_nxt = 0;
            dmem_wen_nxt = 0;
            if (opcode == LW) begin
                immd = instr[BIT_W-1:20];
                dmem_addr = $signed(RS1_DATA) + $signed(immd);
            end
            else if (opcode == SW) begin
                immd = {instr[BIT_W-1:25], instr[11:7]};
                dmem_addr = $signed(RS1_DATA) + $signed(immd);
            end
            else begin
                dmem_addr = 0;
                immd = 0;
            end
            immd = 0;//reset immd
        end
        else if(i_rst_n) begin
            next_PC = PC + 4;
            //decode the cases
            finish = 0;//no finish
            write_to_reg = 0;//no write back to reg
            //mul
            mul_valid = 0;//no multiplication instr
            mul_mode = 0;//initially set to mul
            mul_rs1 = 0;//no mul input
            mul_rs2 = 0;//no mul input
            //decoding
            opcode = instr[6:0];
            FUNC3 = instr[14:12];
            FUNC7 = instr[BIT_W-1:25];
            RS1 = instr[19:15];
            RS2 = instr[24:20];
            RD = instr[11:7];
            //no write data
            WRITE_DATA = 0;
            immd = 0;
            dmem_wdata = 0;
            dmem_rdata = 0;
            dmem_addr = 0;
            //turn off enable signal
            dmem_cen_nxt = 0;
            dmem_wen_nxt = 0;
            case (opcode)
                ASXA: begin
                    //we need further specify the instr
                    case ({FUNC3, FUNC7})
                        {FUNC3_ADD, FUNC7_ADD}: begin
                            //write back to register
                            write_to_reg = 1;
                            //dealing with overflow
                            temp_result = RS1_DATA + RS2_DATA;
                            if ((temp_result[BIT_W-1]==1) && RS1_DATA[BIT_W-1]==0 && RS2_DATA[BIT_W-1]==0) temp_result = 2**31 - 1;//++=-
                            else begin
                                if (temp_result[BIT_W-1]==0 && RS1_DATA[BIT_W-1]==1 && RS2_DATA[BIT_W-1]==1) temp_result = -2**31;//--=+
                                else ;//avoid latch
                            end
                            // WRITE_DATA = temp_result;
                            WRITE_DATA = $signed(RS1_DATA) + $signed(RS2_DATA);
                        end 
                        {FUNC3_SUB, FUNC7_SUB}: begin
                            //write back to register
                            write_to_reg = 1;
                            temp_result = RS1_DATA - RS2_DATA;
                            if ((temp_result[BIT_W-1]==0) && RS1_DATA[BIT_W-1]==1 && RS2_DATA[BIT_W-1]==0) temp_result = -2**31;//-+=+
                            else begin
                                if ((temp_result[BIT_W-1]==1) && RS1_DATA[BIT_W-1]==0 && RS2_DATA[BIT_W-1]==1) temp_result = 2**31-1;//+-=-
                                else ;//avoid latch
                            end
                            // WRITE_DATA = temp_result;
                            WRITE_DATA = $signed(RS1_DATA) - $signed(RS2_DATA);
                        end
                        {FUNC3_AND, FUNC7_AND}: begin
                            //write back to register
                            write_to_reg = 1;
                            WRITE_DATA = RS1_DATA & RS2_DATA;
                        end
                        {FUNC3_XOR, FUNC7_XOR}: begin
                            write_to_reg = 1;
                            WRITE_DATA = RS1_DATA ^ RS2_DATA;
                        end
                        //I found out MUL and DIV opcode are the same
                        {FUNC3_MUL, FUNC7_MUL}: begin
                            write_to_reg = 0;//init no write back to reg
                            mul_rs1 = RS1_DATA;
                            mul_rs2 = RS2_DATA;
                            // $display("here is MUL");
                            //mul ready to write to reg
                            if (mul_ready_output) begin
                                write_to_reg = 1;
                                next_PC = PC + 4;
                                mul_mode = 1;
                                mul_valid = 0;//turn off valid mul unit
                                // $display("here is ready to output");
                            end
                            else begin
                                write_to_reg = 0;//not ready to write
                                next_PC = PC;//stall
                                mul_mode = 0;
                                mul_valid = 1;
                            end
                            WRITE_DATA = mul_wdata[BIT_W-1:0];
                        end
                        {FUNC3_DIV, FUNC7_DIV}: begin
                            write_to_reg = 0;//init no write back to reg
                            mul_rs1 = RS1_DATA;
                            mul_rs2 = RS2_DATA;
                            //mul ready to write to reg
                            if (mul_ready_output) begin
                                write_to_reg = 1;
                                next_PC = PC + 4;
                                mul_mode = 2;
                                mul_valid = 0;//turn off valid mul unit
                            end
                            else begin
                                write_to_reg = 0;//not ready to write
                                next_PC = PC;//stall
                                mul_mode = 0;
                                mul_valid = 1;
                            end
                            WRITE_DATA = mul_wdata[BIT_W-1:0];
                        end
                        default: begin
                            //default setting
                            next_PC = PC + 4;
                            write_to_reg = 0;
                            WRITE_DATA = 0;
                        end 
                    endcase
                end
                IMMD: begin
                    //we need further specify instr
                    case (FUNC3) 
                        FUNC3_ADDI: begin
                            write_to_reg = 1;
                            immd = instr[BIT_W-1:20];
                            //should deal with overflow
                            WRITE_DATA = $signed(RS1_DATA) + $signed(immd);
                        end
                        FUNC3_SLLI: begin
                            write_to_reg = 1;
                            immd = instr[24:20];
                            WRITE_DATA = RS1_DATA << $unsigned(immd);
                        end
                        FUNC3_SLTI: begin
                            write_to_reg = 1;
                            immd = instr[BIT_W-1:20];
                            //if rs1 < immd, then output 1 if not output 0
                            WRITE_DATA = ($signed(RS1_DATA) < $signed(immd))? 1:0;
                        end
                        FUNC3_SRAI: begin
                            write_to_reg = 1;
                            immd = instr[24:20];
                            WRITE_DATA = RS1_DATA >> $unsigned(immd);
                        end
                        default: begin
                            //do nothing
                            next_PC = PC + 4;
                            write_to_reg = 0;
                            WRITE_DATA = 0;
                            immd = 0;
                        end
                    endcase
                end
                LW: begin
                    immd = instr[BIT_W-1:20];
                    dmem_addr = $signed(RS1_DATA) + $signed(immd);
                    //load would have stall, must check it
                    WRITE_DATA = i_DMEM_rdata;
                    write_to_reg = 1;
                    if (stall_counter > 0) begin
                        //write back to reg
                        //turn off enable signal
                        dmem_wen_nxt = 0;
                        dmem_cen_nxt = 0;
                    end
                    else begin
                        //stall
                        //prepare to load data from mem
                        next_PC = PC;
                        //turn on enable signal, no write enable signal
                        dmem_wen_nxt = 0;
                        dmem_cen_nxt = 1;
                    end
                end
                SW: begin
                    immd = {instr[BIT_W-1:25], instr[11:7]};
                    //no need to write back to reg
                    write_to_reg = 0;
                    dmem_addr = $signed(RS1_DATA) + $signed(immd);
                    WRITE_DATA = RS2_DATA;
                    if(stall_counter > 0) begin
                        //turn off enable signal
                        dmem_cen_nxt = 0;
                        dmem_wen_nxt = 0;
                    end
                    else begin
                        //stall
                        next_PC = PC;
                        //turn on enable signal
                        //prepare to write to mem
                        dmem_cen_nxt = 1;
                        dmem_wen_nxt = 1;
                    end
                end
                BRNCH: begin
                    //the position of immd is same
                    immd = {instr[BIT_W-1], instr[7], instr[30:25], instr[11:8], 1'b0};
                    case (FUNC3)
                        FUNC3_BEQ: begin
                            if (RS1_DATA == RS2_DATA) next_PC = $signed(PC) + $signed(immd);//branch
                            else next_PC = PC + 4;
                        end
                        FUNC3_BGE: begin
                            //do signed compared
                            if ($signed(RS1_DATA) >= $signed(RS2_DATA)) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        FUNC3_BLT: begin
                            if ($signed(RS1_DATA) < $signed(RS2_DATA)) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        FUNC3_BNE: begin
                            if (RS1_DATA != RS2_DATA) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        default: next_PC = PC + 4;
                    endcase
                end
                AUIPC: begin
                    write_to_reg = 1;//we need to write back to rd
                    //put usigned 20 bit of immd to leftmost, remaining 12 bits are 0
                    immd = 0;
                    immd[BIT_W-1:12] = instr[BIT_W-1:12];
                    WRITE_DATA = $signed(PC) + $signed(immd);
                end
                JAL: begin
                    //we store the next instr at rd
                    WRITE_DATA = PC+4;
                    write_to_reg = 1;
                    immd = {instr[BIT_W-1], instr[19:12], instr[20], instr[30:21], 1'b0};
                    next_PC = $signed(PC) + $signed({instr[BIT_W-1], instr[19:12], instr[20], instr[30:21], 1'b0});
                    //next_PC cannot directly use immd, weird
                end
                JALR: begin
                    //we store the next instr at rd
                    write_to_reg = 1;
                    WRITE_DATA = PC+4;
                    immd[11:0] = instr[BIT_W-1:20];
                    next_PC = $signed(RS1_DATA) + $signed(immd);
                end
                ECALL: begin
                    write_to_reg = 0;
                    finish = 1; //ready to output
                end
                default: begin
                    next_PC = PC+4;
                    write_to_reg = 0;
                    WRITE_DATA = 0;
                    immd = 0;
                    finish = 0;
                    //mem
                    dmem_addr = 0;
                    dmem_wdata = 0;
                    dmem_rdata = 0;
                    dmem_cen_nxt = 0;
                    dmem_wen_nxt = 0;
                end
            endcase
        end
        else begin
            next_PC = PC + 4;
            finish = 0;
            write_to_reg = 0;
            //mul div
            mul_valid = 0;
            mul_mode = 0;
            mul_rs1 = 0;
            mul_rs2 = 0;
            //decoding
            opcode = instr[6:0];
            FUNC3 = instr[14:12];
            FUNC7 = instr[BIT_W-1:25];
            RS1 = instr[19:15];
            RS2 = instr[24:20];
            RD = instr[11:7];
            WRITE_DATA = 0;
            //data memory
            dmem_wdata = RS2_DATA;
            dmem_rdata = 0;
            dmem_cen_nxt = 0;
            dmem_wen_nxt = 0;
            dmem_addr = 0;
            immd = 0;//reset immd
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
    parameter S_IDLE = 2'b00, S_ONE_CYCLE_OP = 2'b01, S_MULTI_CYCLE_OP = 2'b10;
    parameter IDLE = 0, MUL_MODE = 1, DIV_MODE = 2, SL1_MODE = 3;

    //valid means the muldiv_unit should work now
    input mul_clk, mul_rst_n, mul_valid;
    input [1:0] mul_mode; //mode=0 IDLE, mode=1 MUL, mode=2 DIV
    input [len-1:0] mul_rs1, mul_rs2;
    output [2*len-1:0] mul_data;
    //output the ready signal
    output mul_done;
    
    // Todo: HW2
    //the input would have "mul" as prefix
    reg [2*len-1:0] operand_a;
    reg [2*len-1:0] operand_b;
    reg [1:0] mode, mode_nxt;
    reg [2*len-1:0] out_result; //this would finally connect to output
    reg [2*len:0] mul_temp_result, mul_temp_result_nxt;  //this would deal with overflow prob
    //counter
    reg [4:0] cnt, cnt_nxt;
    reg ready_to_output, ready_to_output_nxt;

    reg [1:0] state, state_nxt;
    
    //wire assignment
    assign mul_done = ready_to_output;
    assign mul_data = out_result;

    //Always Combination
    //load input
    always @(negedge mul_clk) begin
        if (mul_valid && !cnt && mul_rst_n && !ready_to_output) begin
            //multiplication initialization
            mode <= mul_mode;
            mode_nxt <= mul_mode;
            cnt <= 1;
            state <= state_nxt;
            ready_to_output <= 0;//still no ready to output
            mul_temp_result <= mul_temp_result_nxt;
        end
        else if (mul_rst_n) begin
            //if ready to output
            if (cnt == 33) begin
                mode <= 0;
                mode_nxt <= 0;
                mul_temp_result <= mul_temp_result_nxt;
            end
            else begin
                //hold everthin
                mode <= mode_nxt;
                mode_nxt <= mul_mode;
                mul_temp_result <= mul_temp_result_nxt;
            end
            cnt <= cnt_nxt;
            ready_to_output <= ready_to_output_nxt;
            state <= state_nxt;
        end
        else begin
            //reset everything
            mode <= 0;
            mode_nxt <= 0;
            cnt <= 0;
            state <= S_IDLE;
            ready_to_output <= 0;
            mul_temp_result <= 0;
        end
    end
    //FSM
    always @(*) begin
        case(state) 
            S_IDLE: begin
                if (!mul_valid) state_nxt = S_IDLE; //no activate
                else begin
                    case (mul_mode)
                        SL1_MODE: state_nxt = S_ONE_CYCLE_OP;
                        MUL_MODE: state_nxt = S_MULTI_CYCLE_OP;
                        DIV_MODE: state_nxt = S_MULTI_CYCLE_OP;
                        default: state_nxt = S_IDLE;
                    endcase
                end
            end
            S_ONE_CYCLE_OP: begin
                state_nxt = state;//output cycle
            end
            S_MULTI_CYCLE_OP: begin
                if (cnt == 33) begin
                    state_nxt = S_IDLE;//ready to output
                end
                else begin
                    state_nxt = S_MULTI_CYCLE_OP;
                end
            end
            default: begin
                state_nxt = state;
            end
        endcase
    end
    // counter 
    always @(*) begin
        if (state == S_MULTI_CYCLE_OP) begin
            if (cnt == 32) begin
                ready_to_output_nxt = 1;//ready to output
                cnt_nxt = 33;
            end
            else if (cnt == 33) begin
                ready_to_output_nxt = 0;
                cnt_nxt = 0;
            end
            else begin
                ready_to_output_nxt = 0;
                cnt_nxt = cnt + 1;
            end
        end
        else if (state == S_ONE_CYCLE_OP) begin
            //one cycle for output
            ready_to_output_nxt = 1;
            cnt_nxt = 0;
        end
        else begin
            ready_to_output_nxt = 0;
            cnt_nxt = 0;
        end
    end

    //ALU Output
    always @(*) begin
        if (mul_rst_n && mul_valid && cnt > 0) begin
            operand_a = mul_rs1;
            operand_b = mul_rs2;
            case(mode)
                SL1_MODE: begin
                    mul_temp_result_nxt = operand_a << operand_b;
                end
                MUL_MODE: begin
                    if (operand_b[cnt-1]) mul_temp_result_nxt = mul_temp_result + (operand_a <<< (cnt -1));
                    else mul_temp_result_nxt = out_result;
                end
                // DIV_MODE:
                default: mul_temp_result_nxt = 0;
            endcase
        end
        out_result = mul_temp_result;
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

endmodule