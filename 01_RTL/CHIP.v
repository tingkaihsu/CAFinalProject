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
//◆ auipc, jal, jalr
//◆ add, sub, and, xor
//◆ addi, slli, slti, srai
//◆ lw, sw
//◆ mul
//◆ beq, bge, blt, bne
//◆ ecall (the end of program)

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

    //funct3
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

    //funct7
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
        reg [2: 0] funct3;
        reg [6: 0] funct7;
        reg [4: 0] rs1, rs2, rd;
        wire [BIT_W-1: 0] rs1_data, rs2_data;
        reg [BIT_W-1: 0] write_data;
        reg [BIT_W-1: 0] immd;
        reg wrt_to_rd;
        // wire wrt_to_rd;
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
    assign o_finish = finish;
    // assign wrt_to_rd = wrt_to_rd;

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
        .in_A(mul_in_a),
        .in_B(mul_in_b),
        .out(mul_result)
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
        //$display("PC = %d", PC);
        //$display("next_PC = %d", next_PC);
        imem_cen = 1;
        instr = i_IMEM_data;
        next_PC = (i_DMEM_stall)? PC : PC+4;
        if(i_DMEM_stall) begin
            finish = 0;
            wrt_to_rd = 0;
            mem_cen_nxt = 0;
            mem_wen_nxt = 0;
            opcode = instr[6:0];
            funct3 = instr[14:12];
            funct7 = instr[BIT_W-1:25];
            rs1 = instr[19:15];
            rs2 = instr[24:20];
            rd = instr[11:7];
            mul_valid = 0;
            mul_mode = 3;
            mul_in_a = 0;
            mul_in_b = 0;
            wrt_to_rd = 0;
            mem_wdata = rs2_data;
            if(opcode == LW) begin
                immd = instr[BIT_W-1:20];
                dmem_addr = $signed(rs1_data) + $signed(immd);
            end
            else if(opcode == SW) begin
                immd = {instr[BIT_W-1:25], instr[11:7]};
                dmem_addr = $signed(rs1_data) + $signed(immd);
            end
            else begin
                dmem_addr = 0;
            end
            write_data = 0;
            immd = 0;
        end
        else if(i_rst_n)begin
            finish = 0;
            opcode = instr[6:0];
            funct3 = instr[14:12];
            funct7 = instr[BIT_W-1:25];
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
                AUIPC: begin //auipc
                    wrt_to_rd = 1;
                    immd[19:0] = instr[BIT_W-1:12];
                    write_data = PC + {immd[BIT_W-1:12], 12'b0};
                end
                JAL: begin //jal
                    wrt_to_rd = 1;
                    next_PC = $signed(PC) + $signed({instr[BIT_W-1], instr[19:12], instr[20], instr[30:21], 1'b0});
                    write_data = PC + 4;
                end
                JALR: begin //jalr
                    wrt_to_rd = 1;
                    immd[11:0] = instr[BIT_W-1:20];
                    next_PC = $signed(rs1_data) + $signed(immd[11:0]);
                    write_data = PC + 4;
                end
                ASAXM: begin // add, sub, and, xor
                    case({funct3, funct7})
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
                        /*
                        {DIV_FUNCT3, DIV_FUNCT7}: begin
                            wrt_to_rd = 0;
                            mul_in_a = rs1_data;
                            mul_in_b = rs2_data;
                            //$display("mul_valid = %d", mul_valid);
                            if (mul_ready) begin
                                next_PC = PC + 4;
                                wrt_to_rd = 1;
                                mul_mode = 3;
                                mul_valid = 0;
                            end
                            else begin
                                next_PC = PC;
                                mul_mode = 1;
                                mul_valid = 1;
                                wrt_to_rd = 0;
                            end
                            write_data = mul_result[BIT_W-1:0];
                        end
                        */
                        {MUL_FUNCT3, MUL_FUNCT7}: begin
                            wrt_to_rd = 0;
                            mul_in_a = rs1_data;
                            mul_in_b = rs2_data;
                            //$display("mul_valid = %d", mul_valid);
                            if (mul_ready) begin
                                next_PC = PC + 4;
                                wrt_to_rd = 1;
                                mul_mode = 3;
                                mul_valid = 0;
                            end
                            else begin
                                next_PC = PC;
                                mul_mode = 2;
                                mul_valid = 1;
                                wrt_to_rd = 0;
                            end
                            write_data = mul_result[BIT_W-1:0];
                        end
                        default: begin
                            next_PC = PC + 4;
                            wrt_to_rd = 0;
                            write_data = 0;
                        end
                    endcase
                end
                IMMD_OP: begin
                    case (funct3)
                        ADDI_FUNCT3: begin //addi
                            wrt_to_rd = 1;
                            write_data = $signed(rs1_data) + $signed(instr[BIT_W-1:20]);
                        end
                        SLLI_FUNCT3: begin //slli
                            wrt_to_rd = 1;
                            write_data = rs1_data << $unsigned(instr[24:20]);
                        end
                        SLTI_FUNCT3: begin //slti
                            wrt_to_rd = 1;
                            write_data = ($signed(rs1_data) < $signed(instr[BIT_W-1:20]))? 1 : 0;
                        end
                        SRAI_FUNCT3: begin //srai
                            wrt_to_rd = 1;
                            write_data = rs1_data >> $unsigned(instr[24:20]);
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
                    immd = instr[BIT_W-1:20];
                    if(!i_DMEM_stall && stall_counter > 0) begin
                        wrt_to_rd = 1;
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 0;
                        write_data = i_DMEM_rdata;
                        // $display("load value %h", write_data);
                    end
                    else begin
                        dmem_addr = $signed(rs1_data) + $signed(immd);
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 1;
                        next_PC = PC;
                    end
                    
                end
                SW: begin
                    wrt_to_rd = 0;
                    immd = {instr[BIT_W-1:25], instr[11:7]};
                    if(!i_DMEM_stall && stall_counter > 0) begin
                        dmem_addr = $signed(rs1_data) + $signed(immd);
                        mem_wdata = rs2_data;
                        mem_wen_nxt = 0;
                        mem_cen_nxt = 0;
                    end
                    else begin
                        dmem_addr = $signed(rs1_data) + $signed(immd);
                        mem_wdata = rs2_data;
                        mem_cen_nxt = 1;
                        mem_wen_nxt = 1;
                        next_PC = PC;
                    end
                end
                BRNCH: begin //beq, bge, blt, bne
                    immd = {instr[BIT_W-1], instr[7], instr[30:25], instr[11:8], 1'b0};
                    case(funct3)
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
                            else begin
                                next_PC = PC + 4;
                            end
                        end
                        default: begin
                            next_PC = PC + 4;
                        end
                    endcase
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
            next_PC = 0;
            finish = 0;
            opcode = 0;
            funct3 = 0;
            funct7 = 0;
            rs1 = 0;
            rs2 = 0;
            rd = 0;
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

module MULDIV_unit(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    parameter BIT_W = 32;
    input clk, rst_n, valid;
    input [1:0] mode; // 0: shift left, 1: div, 2: mul, 3:IDLE
    output ready;
    input [BIT_W-1:0] in_A, in_B;
    output [2*BIT_W-1:0] out;

    // definition of state
    reg [1:0] state, state_nxt;
    parameter S_IDLE = 2'b00, S_ONE_CYCLE_OP = 2'b01, S_MULTI_CYCLE_OP = 2'b10;

    // definition of internal signals
    reg [2*BIT_W-1:0] alu_out, operand_a, operand_b;
    reg [5:0] counter, counter_nxt;
    reg rdy, rdy_nxt;
    reg [1:0] mode_now, mode_nxt;
    reg [2*BIT_W-1: 0] temp, temp_nxt;
    assign ready = rdy;
    assign out = alu_out;

    always @(negedge clk) begin
        if (valid && counter == 0 && rst_n && rdy == 0) begin
            mode_now <= mode;
            mode_nxt <= mode;
            counter <= 1;
            state <= state_nxt;
            rdy <= 0;
            temp <= temp_nxt;
        end
        else if(rst_n)begin
            if (counter == 33) begin
                mode_now <= 3;
                mode_nxt <= 3;
                temp <= temp_nxt;
            end
            else begin
                mode_now <= mode_nxt;
                mode_nxt <= mode;
                temp <= temp_nxt;
            end
            counter <= counter_nxt;
            state <= state_nxt;
            rdy <= rdy_nxt;
        end
        else begin
            mode_now <= 3;
            mode_nxt <= 3;
            counter <= 0;
            state <= S_IDLE;
            rdy <= 0;
            temp <= 0;
        end
    end

    always @(*) begin
        case(state)
            S_IDLE: begin
                if(!valid) state_nxt = S_IDLE;
                else begin
                    case(mode)
                        2'b00: begin
                            state_nxt = S_ONE_CYCLE_OP;
                        end
                        2'b01: begin
                            state_nxt = S_MULTI_CYCLE_OP;
                        end
                        2'b10: begin
                            state_nxt = S_MULTI_CYCLE_OP;
                        end
                        default: begin
                            state_nxt = S_IDLE;
                        end
                    endcase
                end
            end
            S_ONE_CYCLE_OP: state_nxt = S_IDLE;
            S_MULTI_CYCLE_OP: begin
                if(counter == 33) state_nxt = S_IDLE;
                else state_nxt = S_MULTI_CYCLE_OP;
            end
            default: state_nxt = state;
        endcase
    end

    always @(*) begin
        if (state == S_MULTI_CYCLE_OP) begin
            if(counter == 32) begin
                rdy_nxt = 1;
                counter_nxt = 33;
            end
            else if(counter > 32) begin
                rdy_nxt = 0;
                counter_nxt = 0;
            end
            else begin
                rdy_nxt = 0;
                counter_nxt = counter + 1;
            end
        end
        else if (state == S_ONE_CYCLE_OP) begin
            rdy_nxt = 1;
            counter_nxt = 0;
        end
        else begin
            rdy_nxt = 0;
            counter_nxt = 0;
        end
    end

    always @(*) begin
        if(rst_n && valid && counter > 0) begin
            operand_a = in_A;
            operand_b = in_B;
            case(mode_now)
                2'b00: begin
                    temp_nxt = operand_a << operand_b;
                end
                /*
                2'b01: begin//div
                    if(counter == 1) begin
                        temp = {32'h0000_0000, operand_a[BIT_W-1:0]};
                        //temp_1 = {operand_b[BIT_W-1:0], 32'h0000_0000};
                        if({32'h0000_0000, operand_a[30:0], 1'b0} >= {operand_b[BIT_W-1:0], 32'h0000_0000}) begin
                            temp_nxt = temp - //temp_1 + 1'b1;
                        end
                        else begin
                            temp_nxt = temp;
                        end
                    end
                    else begin
                        //temp_1 = //temp_1;
                        temp = {temp[62:0], 1'b0};
                        if(temp >= //temp_1) begin
                            $display ("temp = %d", temp);
                            $display ("//temp_1 = %d", //temp_1);
                            temp_nxt = temp - //temp_1 + 1'b1;
                        end
                        else begin
                            temp_nxt = temp;
                        end
                    end
                end
                */
                2'b10: begin
                    //$display("counter = %d", (counter-1));
                    //$display("operand_b[counter] = %d", operand_b[(counter-1)]);
                    //$display("operand_a = %d", operand_a);
                    if(operand_b[(counter-1)]) begin
                        temp_nxt = temp + (operand_a <<< (counter-1));
                    end
                    else begin
                        temp_nxt = alu_out;
                    end
                end
                default: begin
                    temp_nxt = 64'h0000_0000_0000_0000;
                end
            endcase
        end
        else begin
            temp_nxt = 64'h0000_0000_0000_0000;
        end
        alu_out = temp;
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