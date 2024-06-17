//final version
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

    // TODO: any declaration
    //opcode
    parameter AUIPC = 7'b0010111;
    parameter JAL = 7'b1101111;
    parameter JALR = 7'b1100111;
    parameter ASAXM = 7'b0110011;//ADD SUB AND XOR MUL
    parameter IMMD_OP = 7'b0010011;
    parameter LW = 7'b0000011;
    parameter SW = 7'b0100011;
    parameter DIV = 7'b0110011;
    parameter BRNCH = 7'b1100011;
    parameter ECALL = 7'b1110011;

    //func3
    parameter JALR_FUNCT3 = 3'b000;
    parameter ADD_FUNCT3 = 3'b000;
    parameter SUB_FUNCT3 = 3'b000;
    parameter AND_FUNCT3 = 3'b111;
    parameter XOR_FUNCT3 = 3'b100;
    parameter ADDI_FUNCT3 = 3'b000;
    parameter SLLI_FUNCT3 = 3'b001;
    parameter SLTI_FUNCT3 = 3'b010;
    parameter SRAI_FUNCT3 = 3'b101;
    parameter LW_FUNCT3 = 3'b010;
    parameter SW_FUNCT3 = 3'b010;
    parameter MUL_FUNCT3 = 3'b000;
    parameter DIV_FUNCT3 = 3'b100;
    parameter BEQ_FUNCT3 = 3'b000;
    parameter BGE_FUNCT3 = 3'b101;
    parameter BLT_FUNCT3 = 3'b100;
    parameter BNE_FUNCT3 = 3'b001;
    parameter ECALL_FUNCT3 = 3'b000;

    //func7
    parameter ADD_FUNCT7 = 7'b0000000;
    parameter SUB_FUNCT7 = 7'b0100000;
    parameter XOR_FUNCT7 = 7'b0000000;
    parameter SLLI_FUNCT7 = 7'b0000000;
    parameter SRAI_FUNCT7 = 7'b0100000;
    parameter MUL_FUNCT7 = 7'b0000001;
    parameter DIV_FUNCT7 = 7'b0000001;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        reg mem_cen, mem_wen, imem_cen, mem_cen_nxt, mem_wen_nxt;
        reg [BIT_W-1:0] dmem_addr, mem_wdata, mem_rdata;

        reg [6: 0] opcode;
        reg [2: 0] func3;
        reg [6: 0] func7;
        reg [4: 0] rs1, rs2, rd;
        wire [BIT_W-1: 0] rs1_data, rs2_data;
        reg [BIT_W-1: 0] write_data;
        reg [BIT_W-1: 0] immd;
        reg wrt_to_rd;
        wire mul_ready; //for multi-cycle operation
        reg mul_valid;
        reg [1: 0] mul_mode;
        wire [2*BIT_W-1: 0] mul_result;
        reg [BIT_W-1: 0] mul_in_a, mul_in_b;
        reg [BIT_W-1: 0] instr;
        reg finish;
        reg [BIT_W-1:0] stall_counter;
        


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = PC;
    assign o_IMEM_cen = imem_cen;
    assign o_DMEM_cen = mem_cen;
    assign o_DMEM_wen = mem_wen;
    assign o_DMEM_addr = dmem_addr;
    assign o_DMEM_wdata = mem_wdata;
    //if no cache
    // assign o_finish = finish;
    assign o_finish = i_cache_finish;
    assign o_proc_finish = finish;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (wrt_to_rd),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (write_data),             
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );

    MULDIV_unit muldiv_unit(
        .clk(i_clk),
        .rst_n(i_rst_n),
        .valid(mul_valid),
        .ready(mul_ready),
        .mode(mul_mode),
        .rs1_data(mul_in_a),
        .rs2_data(mul_in_b),
        .rd_data(mul_result)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC = 32'h00010000; // Do not modify this value!!!
            mem_cen = 0;
            mem_wen = 0;
        end
        else begin
            PC = next_PC;
            mem_cen = mem_cen_nxt;
            mem_wen = mem_wen_nxt;
        end
    end

    always @(posedge i_clk) begin
        if ((opcode == LW || opcode == SW) && i_DMEM_stall == 1) begin
            stall_counter = stall_counter + 1;
        end
        else begin
            stall_counter = 0;
        end
    end

    always @(*) begin
        imem_cen = 1;
        instr = i_IMEM_data;
        next_PC = (i_DMEM_stall)? PC : PC+4;
        if (!i_DMEM_stall) begin
            finish = 0;
            opcode = instr[6:0];
            func3 = instr[14:12];
            func7 = instr[BIT_W-1:25];
            rs1 = instr[19:15];
            rs2 = instr[24:20];
            rd = instr[11:7];
            wrt_to_rd = 0;
            mul_valid = 0;
            mul_mode = 3;
            mul_in_a = 0;
            mul_in_b = 0;
            dmem_addr = 0;
            mem_wdata = 0;
            mem_rdata = 0;
            mem_cen_nxt = 0;
            mem_wen_nxt = 0;
            write_data = 0;
            immd = 0;
            case(opcode) 
                ASAXM: begin // add, sub, and, xor, mul
                    case({func3, func7})
                        {ADD_FUNCT3, ADD_FUNCT7}: begin
                            wrt_to_rd = 1;
                            write_data = $signed(rs1_data) + $signed(rs2_data);
                            // dealing with overflow
                        end
                        {SUB_FUNCT3, SUB_FUNCT7}: begin
                            wrt_to_rd = 1;
                            write_data = $signed(rs1_data) - $signed(rs2_data);
                        end
                        {AND_FUNCT3, ADD_FUNCT7}: begin
                            wrt_to_rd = 1;
                            write_data = rs1_data & rs2_data;
                        end
                        {XOR_FUNCT3, XOR_FUNCT7}: begin
                            wrt_to_rd = 1;
                            write_data = rs1_data ^ rs2_data;
                        end
                        {MUL_FUNCT3, MUL_FUNCT7}: begin
                            wrt_to_rd = 0;
                            mul_in_a = rs1_data;
                            mul_in_b = rs2_data;
                            if (mul_ready) begin
                                next_PC = PC + 4;
                                wrt_to_rd = 1;
                                mul_mode = 0;//IDLE
                                mul_valid = 0;//deactivate muldiv unit
                            end
                            else begin
                                next_PC = PC;//stall
                                mul_mode = 1;//MUL
                                mul_valid = 1;//activate muldiv unit
                                wrt_to_rd = 0;
                            end
                            write_data = mul_result[BIT_W-1:0];
                        end
                        // {DIV_FUNCT3, DIV_FUNCT7}: begin
                        //     wrt_to_rd = 0;
                        //     mul_in_a = rs1_data;
                        //     mul_in_b = rs2_data;
                        //     //$display("mul_valid = %d", mul_valid);
                        //     if (mul_ready) begin
                        //         next_PC = PC + 4;
                        //         wrt_to_rd = 1;
                        //         mul_mode = 0;//IDLE
                        //         mul_valid = 0;//deactivate
                        //     end
                        //     else begin
                        //         next_PC = PC;
                        //         mul_mode = 2;//DIV
                        //         mul_valid = 1;//activate MULDIV unit
                        //         wrt_to_rd = 0;
                        //     end
                        //     write_data = mul_result[BIT_W-1:0];
                        // end
                        default: begin
                            next_PC = PC + 4;
                            wrt_to_rd = 0;
                            write_data = 0;
                        end
                    endcase
                end
                IMMD_OP: begin
                    case (func3)
                        ADDI_FUNCT3: begin //addi
                            wrt_to_rd = 1;
                            immd[11:0] = instr[BIT_W-1:20];
                            immd[BIT_W-1:12] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;
                            // write_data = $signed(rs1_data) + $signed(instr[BIT_W-1:20]);
                            write_data = $signed(rs1_data) + $signed(immd);
                        end
                        SLLI_FUNCT3: begin //slli
                            wrt_to_rd = 1;
                            immd[4:0] = instr[24:20];
                            immd[BIT_W-1:5] = (instr[24])? 27'b111111111111111111111111111 : 0;
                            write_data = rs1_data << $unsigned(instr[24:20]);//seems don't need signed extension
                            // write_data = rs1_data << $unsigned(immd);
                        end
                        SLTI_FUNCT3: begin //slti
                            wrt_to_rd = 1;
                            immd[11:0] = instr[BIT_W-1:20];
                            immd[BIT_W-1:12] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;
                            // write_data = ($signed(rs1_data) < $signed(instr[BIT_W-1:20]))? 1 : 0;
                            write_data = ($signed(rs1_data) < $signed(immd))? 1 : 0;
                        end
                        SRAI_FUNCT3: begin //srai
                            wrt_to_rd = 1;
                            immd[4:0] = instr[24:20];
                            immd[BIT_W-1:5] = (instr[24])? 27'b111111111111111111111111111 : 0;
                            write_data = rs1_data >> $unsigned(instr[24:20]);//seems don't need sign extension
                        end
                        default: begin
                            next_PC = PC + 4;
                            wrt_to_rd = 0;
                            write_data = 0;
                            immd = 0;
                        end
                    endcase
                end
                LW: begin 
                    immd[11:0] = instr[BIT_W-1:20];
                    immd[BIT_W-1:12] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;
                    dmem_addr = $signed(rs1_data) + $signed(immd);
                    write_data = i_DMEM_rdata;
                    wrt_to_rd = 1;
                    if(stall_counter > 0) begin
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 0;
                    end
                    else begin
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 1;
                        next_PC = PC;
                    end
                end
                SW: begin
                    immd = {instr[BIT_W-1:25], instr[11:7]};
                    immd[BIT_W-1:12] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;
                    dmem_addr = $signed(rs1_data) + $signed(immd);
                    mem_wdata = rs2_data;
                    wrt_to_rd = 0;
                    if(stall_counter > 0) begin
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 0;
                    end
                    else begin
                        mem_cen_nxt = 1;
                        mem_wen_nxt = 1;
                        next_PC = PC;
                    end
                end
                BRNCH: begin //beq, bge, blt, bne
                    immd = {instr[BIT_W-1], instr[7], instr[30:25], instr[11:8], 1'b0};
                    immd[BIT_W-1:13] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;
                    case(func3)
                        BEQ_FUNCT3: begin //beq
                            if(rs1_data == rs2_data) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        BGE_FUNCT3: begin //bge
                            if($signed(rs1_data) >= $signed(rs2_data)) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        BLT_FUNCT3: begin //blt
                            if($signed(rs1_data) < $signed(rs2_data)) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        BNE_FUNCT3: begin //bne
                            if(rs1_data != rs2_data) next_PC = $signed(PC) + $signed(immd);
                            else next_PC = PC + 4;
                        end
                        default: begin
                            next_PC = PC + 4;
                        end
                    endcase
                end
                AUIPC: begin
                    wrt_to_rd = 1;
                    // immd[19:0] = instr[BIT_W-1:12];
                    // write_data = PC + {immd[BIT_W-1:12], 12'b0};
                    immd[BIT_W-1:12] = instr[BIT_W-1:12];
                    immd[11:0] = 12'b0;
                    write_data = PC + immd;
                end
                JAL: begin
                    wrt_to_rd = 1;
                    immd = {instr[BIT_W-1], instr[19:12], instr[20], instr[30:21], 1'b0};
                    immd[BIT_W-1:21] = (instr[BIT_W-1])? 11'b11111111111:0;//sign extension
                    // next_PC = $signed(PC) + $signed({instr[BIT_W-1], instr[19:12], instr[20], instr[30:21], 1'b0});
                    next_PC = $signed(PC) + $signed(immd);
                    write_data = PC + 4;
                end
                JALR: begin
                    wrt_to_rd = 1;
                    immd[11:0] = instr[BIT_W-1:20];
                    immd[BIT_W-1:12] = (instr[BIT_W-1])? 20'b11111111111111111111 : 0;//signed extension
                    // next_PC = $signed(rs1_data) + $signed(immd[11:0]);
                    next_PC = $signed(rs1_data) + $signed(immd);
                    write_data = PC + 4;
                end
                ECALL: begin //ecall
                    wrt_to_rd = 0;
                    finish = 1;
                end
                default: begin
                    next_PC = PC + 4;
                    wrt_to_rd = 0;
                    write_data = 0;
                    immd = 0;
                    finish = 0;
                    dmem_addr = 0;
                    mem_wdata = 0;
                    mem_rdata = 0;
                    mem_cen_nxt = 0;
                    mem_wen_nxt = 0;
                end
            endcase
        end
        else begin
            next_PC = PC;
            finish = 0;
            wrt_to_rd = 0;
            mul_valid = 0;
            mul_mode = 3;
            mul_in_a = 0;
            mul_in_b = 0;
            
            opcode = instr[6:0];
            func3 = instr[14:12];
            func7 = instr[BIT_W-1:25];
            rs1 = instr[19:15];
            rs2 = instr[24:20];
            rd = instr[11:7];
            write_data = 0;

            mem_wdata = rs2_data;
            mem_rdata = 0;
            mem_cen_nxt = 0;
            mem_wen_nxt = 0;
            if (opcode == LW) begin
                immd = instr[BIT_W-1:20];
                dmem_addr = $signed(rs1_data) + $signed(immd);
            end
            else if (opcode == SW) begin
                immd = {instr[BIT_W-1:25], instr[11:7]};
                dmem_addr = $signed(rs1_data) + $signed(immd);
            end
            else begin
                immd = 0;
                dmem_addr = 0;
            end
            immd = 0;
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

module MULDIV_unit(clk, rst_n, valid, ready, mode, rs1_data, rs2_data, rd_data);
    // Todo: your HW2
    // Definition of ports
    parameter BIT_W = 32;
    parameter S_IDLE = 2'b00, S_ONE_CYCLE_OP = 2'b01, S_MULTI_CYCLE_OP = 2'b10;
    parameter MUL_MODE = 2'b01, IDLE = 2'b00, DIV_MODE = 2'b10;
    
    input clk, rst_n, valid;
    input [1:0] mode; // 0: IDLE, 1: MUL, 2: DIV
    output ready;
    input [BIT_W-1:0] rs1_data, rs2_data;
    output [2*BIT_W-1:0] rd_data;

    // definition of state
    reg [1:0] state, state_nxt;

    // definition of internal signals
    reg [2*BIT_W-1:0] alu_out, operand_a, operand_b;
    reg [5:0] cnt, cnt_nxt;
    reg rdy, rdy_nxt;
    reg [1:0] mode_now, mode_nxt;
    reg [2*BIT_W-1: 0] tmp, tmp_nxt;
    assign ready = rdy;
    assign rd_data = alu_out;

    always @(negedge clk) begin
        if (valid && !cnt && rst_n && !rdy) begin
            mode_now <= mode;
            mode_nxt <= mode;
            cnt <= 1;
            state <= state_nxt;
            rdy <= 0;
            tmp <= tmp_nxt;
        end
        else if(rst_n)begin
            if (cnt == 33) begin
                mode_now <= 0;//idle mode
                mode_nxt <= 0;//idle mode
                tmp <= tmp_nxt;
            end
            else begin
                mode_now <= mode_nxt;
                mode_nxt <= mode;
                tmp <= tmp_nxt;
            end
            cnt <= cnt_nxt;
            state <= state_nxt;
            rdy <= rdy_nxt;
        end
        else begin
            mode_now <= 0;//idle mode
            mode_nxt <= 0;//idle mode
            cnt <= 0;
            state <= S_IDLE;
            rdy <= 0;
            tmp <= 0;
        end
    end

    always @(*) begin
        case(state)
            S_IDLE: begin
                if(!valid) state_nxt = S_IDLE;
                else begin
                    case(mode)
                        // DIV_MODE: begin
                        //     state_nxt = S_MULTI_CYCLE_OP;
                        // end
                        MUL_MODE: begin
                            state_nxt = S_MULTI_CYCLE_OP;
                        end
                        default: begin
                            state_nxt = S_IDLE;
                        end
                    endcase
                end
            end
            // S_ONE_CYCLE_OP: state_nxt = S_IDLE;
            S_MULTI_CYCLE_OP: begin
                if(cnt == 33) state_nxt = S_IDLE;//cnt limit
                else state_nxt = S_MULTI_CYCLE_OP;
            end
            default: state_nxt = state;
        endcase
    end

    always @(*) begin
        if (state == S_MULTI_CYCLE_OP) begin
            if(cnt == 32) begin
                //ready to output
                rdy_nxt = 1;
                cnt_nxt = 33;
            end
            else if(cnt > 32) begin
                //illegal state, reset
                rdy_nxt = 0;
                cnt_nxt = 0;
            end
            else begin
                //increase cnt
                rdy_nxt = 0;
                cnt_nxt = cnt + 1;
            end
        end
        else if (state == S_ONE_CYCLE_OP) begin
            //ready to output and reset cnt
            rdy_nxt = 1;
            cnt_nxt = 0;
        end
        else begin
            rdy_nxt = 0;
            cnt_nxt = 0;
        end
    end

    always @(*) begin
        if(rst_n && valid && cnt > 0) begin
            operand_a = rs1_data;
            operand_b = rs2_data;
            case(mode_now)
                MUL_MODE: begin
                    if(operand_b[cnt-1]) begin//if the operand_b[idx] bit is 1, we shift the operand_a idx bit 
                        tmp_nxt = tmp + (operand_a << (cnt-1));//idx = cnt - 1
                    end
                    else begin
                        tmp_nxt = tmp;//if the operand_b[idx] is 0
                    end
                end
                default: begin
                    tmp_nxt = 64'h0;
                end
            endcase
        end
        else begin
            tmp_nxt = 64'h0;//reset tmp_nxt
        end
        alu_out = tmp;//output time
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32//address width
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
        //memory interface
        output o_mem_cen,
        output o_mem_wen,
        output [ADDR_W-1:0] o_mem_addr,
        output [BIT_W*4-1:0]  o_mem_wdata,
        input [BIT_W*4-1:0] i_mem_rdata,
        input i_mem_stall,
        output o_cache_available,
        //offset
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    // //------------------------------------------//
    // //          default connection              //
    // assign o_mem_cen = i_proc_cen;              //
    // assign o_mem_wen = i_proc_wen;              //
    // assign o_mem_addr = i_proc_addr;            //
    // assign o_mem_wdata = i_proc_wdata;          //
    // assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    // assign o_proc_stall = i_mem_stall;          //
    // //------------------------------------------//
    // Todo: BONUS
    //parameters:
    parameter S_IDLE = 3'b00;//000
    parameter S_FIND = 3'b01;//001
    parameter S_WB = 3'b10;//010
    parameter S_ALLO = 3'b011;//011
    parameter S_FINISH = 3'b100;//100
    parameter block_number = 16;//optimal block size

    //regs
    reg [2:0] state, state_nxt;
    reg hit;//record if hit or not
    reg mem_cen, mem_wen;
    reg cen, cen_nxt;
    reg wen, wen_nxt;
    reg proc_stall;//stall the process
    reg [4*BIT_W-1:0] data[0:block_number-1], data_nxt[0:block_number-1];
    reg [23:0] tag[0:block_number-1], tag_nxt[0:block_number-1];
    //valid bit record if there is data in cache or not
    reg valid[0:block_number-1], valid_nxt[0:block_number-1];
    //dirty bit record if we need to write back to data mem or not
    reg dirty[0:block_number-1], dirty_nxt[0:block_number-1];
    reg [3:0] idx, index_nxt;
    reg [1:0] offset, offset_nxt;
    reg [3:0] cnt;
    //finish bit to tell the processor cache search is finished
    reg finish;
    reg [ADDR_W-1:0] proc_addr, real_addr;

    //wire assignment
    assign o_mem_cen = mem_cen;
    assign o_mem_wen = mem_wen;
    assign o_mem_addr = proc_addr + i_offset;
    assign o_mem_wdata = data[idx];
    assign o_proc_stall = proc_stall;

    wire [ADDR_W-1:0] selected_data;
    assign selected_data = data[idx] >> (ADDR_W * offset);
    assign o_proc_rdata = selected_data[ADDR_W-1:0];

    //tell the processor that the cache finish
    assign o_cache_finish = finish;

    //always blocks
    always @(*) begin
        real_addr = i_proc_addr - i_offset;
        if (state == S_WB) begin
            idx = real_addr[7:4];
            offset = 0;
            //2'b00 is byte offset
            proc_addr = {tag[idx], idx, offset, 2'b00};
        end
        else if (state == S_FINISH) begin
            idx = cnt;
            offset = 0;
            //2'b00 is byte offset
            proc_addr = {tag[idx], idx, offset, 2'b00};
        end
        else begin 
            proc_addr = real_addr;
            idx = proc_addr[7:4];
            offset = proc_addr[3:2];
        end
        //if the addr is correct and valid bit is high, hit!
        hit = (tag[idx] == proc_addr[31:8]) & valid[idx];
    end

    //FSM
    always @(*) begin
        case(state)
            S_IDLE: begin
                finish = 0;
                if (i_proc_finish) state_nxt = S_FINISH;//ready to finish
                else begin
                    if (i_proc_cen) state_nxt = S_FIND;
                    else state_nxt = S_IDLE;
                end
            end
            S_FIND: begin
                finish = 0;
                if(hit) state_nxt = S_IDLE;
                else begin
                    if (dirty[idx]) begin
                        state_nxt = S_WB;
                    end
                    else state_nxt = S_ALLO;
                end 
            end
            S_WB:   begin
                finish = 0;
                if(!i_mem_stall) begin
                    state_nxt = S_ALLO;
                end
                else state_nxt = S_WB;
            end
            S_ALLO: begin
                finish = 0;
                if(!i_mem_stall) begin
                    state_nxt = S_FIND;
                end
                else state_nxt = S_ALLO;
            end
            S_FINISH: begin
                if (cnt == (block_number-1) && i_mem_stall == 0) begin
                    //cnt == block_number means all data memory is stored
                    state_nxt = S_IDLE;
                    finish = 1;//ready to finish
                end
                else begin
                    state_nxt = S_FINISH;
                    finish = 0;
                end
            end
            default: begin
                state_nxt = state;
                finish = 0;
            end
        endcase
    end

    integer i;
    always @(*) begin
        //for loop
        for (i = 0; i < block_number ; i = i + 1) begin
            dirty_nxt[i] = 0;
            tag_nxt[i] = 0;
            valid_nxt[i] = 0;
            data_nxt[i] = 0;
        end
        case(state)
            S_IDLE: begin
                dirty_nxt[idx] = dirty[idx];
                mem_cen = 0;
                mem_wen = 0;
                proc_stall = i_proc_cen;
                cen_nxt = i_proc_cen;
                wen_nxt = i_proc_wen;
                tag_nxt[idx] = tag[idx];
                valid_nxt[idx] = valid[idx];
                data_nxt[idx] = data[idx];
            end
            S_FIND: begin
                mem_cen = 0;
                mem_wen = 0;
                cen_nxt = cen;
                wen_nxt = wen;
                tag_nxt[idx] = tag[idx];
                valid_nxt[idx] = valid[idx];
                if (hit) begin
                    if (wen) begin
                        dirty_nxt[idx] = 1;
                        data_nxt[idx] = data[idx];
                        // data_nxt[idx][ADDR_W*offset+ADDR_W-1 : ADDR_W] = i_proc_wdata;
                        //but position is variables, so we need special syntax
                        data_nxt[idx][ADDR_W*offset +: ADDR_W] = i_proc_wdata;
                    end
                    else begin
                        data_nxt[idx] = data[idx];
                        dirty_nxt[idx] = dirty[idx]; 
                    end
                    proc_stall = 0;
                end
                else begin
                    proc_stall = 1;
                    data_nxt[idx] = data[idx];
                    if(wen) dirty_nxt[idx] = 1;//dirty bit is high
                    else dirty_nxt[idx] = dirty[idx];//hold the dirty bit
                end
            end
            S_WB: begin
                //write back stage
                data_nxt[idx] = data[idx];
                dirty_nxt[idx] = dirty[idx];
                mem_cen = 1;
                mem_wen = 1;
                proc_stall = 1;
                cen_nxt = cen;
                wen_nxt = wen;
                tag_nxt[idx] = tag[idx];
                valid_nxt[idx] = valid[idx];
            end
            S_ALLO: begin
                //allocation stage
                data_nxt[idx] = i_mem_rdata;
                dirty_nxt[idx] = dirty[idx];
                mem_cen = 1;
                mem_wen = 0;
                proc_stall = 1;
                cen_nxt = cen;
                wen_nxt = wen;
                tag_nxt[idx] = proc_addr[31:8];
                valid_nxt[idx] = 1;
            end
            S_FINISH: begin
                //finish stage
                data_nxt[idx] = data[idx];
                cen_nxt = cen;
                wen_nxt = wen;
                tag_nxt[idx] = tag[idx];
                valid_nxt[idx] = valid[idx];
                if (dirty[idx]) begin
                    //allow to write back mem
                    mem_cen = 1;
                    mem_wen = 1;
                end
                else begin
                    mem_cen = 0;
                    mem_wen = 0;    
                end
                //pull low dirty bit
                dirty_nxt[idx] = 0;
                proc_stall = 1;
            end
            default: begin
                data_nxt[idx] = 0;
                dirty_nxt[idx] = dirty[idx];
                mem_cen = 0;
                mem_wen = 0;
                proc_stall = 0;
                cen_nxt = 0;
                wen_nxt = 0;
                tag_nxt[idx] = tag[idx];
                valid_nxt[idx] = valid[idx];
            end
        endcase
    end

    //counter
    always @(posedge i_clk) begin
        if (state == S_FINISH) begin
            //only count at finish state
            //only increase when there is no stall
            if (!i_mem_stall) cnt <= cnt + 1;
            else cnt <= cnt;
        end
        else cnt <= 0;
    end

    //sequential block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            //reset everything
            state <= S_IDLE;
            //for loop to interate over all data and clear up
            for (i = 0; i < block_number; i = i+1)begin
                data[i] <= 0;
                valid[i] <= 0;
                tag[i] <= 0;
                dirty[i] <= 0;
            end
            cen <= 0;
            wen <= 0; 
        end
        else begin
            //hold everything
            state <= state_nxt;
            if (cen) begin
                data[idx] <= data_nxt[idx];
                valid[idx] <= valid_nxt[idx];
                tag[idx] <= tag_nxt[idx];
                dirty[idx] <= dirty_nxt[idx];
            end
            cen <= cen_nxt;
            wen <= wen_nxt;
        end
    end
endmodule