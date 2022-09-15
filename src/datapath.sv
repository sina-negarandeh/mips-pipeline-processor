module MIPS(clk,rst);
  input clk, rst;
  wire regwrite, alusrc, regdst, memwrite, memread;
  wire memtoreg;
  wire [2:0] alu_op;
  wire [5:0] OPC;
  wire [5:0] func;
  wire [31:0] instruction;
  wire [4:0] id_ex_reg_rt;
  wire [4:0] id_dx_reg_rs;
  wire [4:0] ex_mem_reg_rd;
  wire [4:0] mem_wb_reg_rd;
  wire ex_mem_regwrite, mem_wb_regwrite;
  wire [1:0] forward_a;
  wire [1:0] forward_b;
  wire pcwrite, if_id_write, mux_select;
  wire [4:0] if_id_reg_rs;
  wire [4:0] if_id_reg_rt;
  wire id_ex_memread;
  wire [1:0] forward_c;
  wire [1:0] forward_d;
  
  wire ex_mem_memread;
  wire id_ex_regwrite;
  wire [4:0] if_id_rs;
  wire [4:0] if_id_rt;
  wire [4:0] id_ex_rd;
  wire [4:0] ex_mem_rt;
  
  wire equal, if_id_reg_flush;
  wire [1:0] pcsrc;
  
  
  datapath DP (clk, rst, regwrite, alusrc, alu_op, regdst, memwrite, memread, memtoreg, forward_a, forward_b, id_ex_reg_rt, id_dx_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, instruction, pcwrite, if_id_write, mux_select, if_id_reg_rs, if_id_reg_rt, id_ex_memread, equal, pcsrc, if_id_reg_flush, if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, forward_c, forward_d);
  ControlUnit CU (rst,instruction[31:26], instruction[5:0], id_ex_reg_rt, id_dx_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, if_id_reg_rs, if_id_reg_rt, id_ex_memread, regdst, alusrc, memtoreg, regwrite, memread, memwrite, alu_op, forward_a, forward_b, pcwrite, if_id_write, mux_select, equal, pcsrc, if_id_reg_flush, if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, forward_c, forward_d);
endmodule

//pcwrite, if_id_write, mux_select input signals added
//if_id_reg_rs, if_id_reg_rt, id_ex_memread output signals added
//pc and if/id registers enable signal added
//mux for id/ex registers added
module datapath (clk, rst, regwrite, alusrc, alu_op, regdst, memwrite, memread, memtoreg, forward_a, forward_b, id_ex_reg_rt, id_ex_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, instruction_reg, pcwrite, if_id_write, mux_select, if_id_reg_rs, if_id_reg_rt, id_ex_memread, equal, pcsrc, if_id_reg_flush, if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, forward_c, forward_d);
  input clk, rst;
  input regwrite, alusrc, regdst, memwrite, memread, memtoreg;
  input [2:0] alu_op;
  output [31:0] instruction_reg;
  
  input [1:0] forward_a;
  input [1:0] forward_b;
  input [1:0] forward_c;
  input [1:0] forward_d;
  
  output [4:0] id_ex_reg_rt;
  output [4:0] id_ex_reg_rs;
  output [4:0] ex_mem_reg_rd;
  output [4:0] mem_wb_reg_rd;
  output ex_mem_regwrite, mem_wb_regwrite;
  
  output ex_mem_memread;
  output id_ex_regwrite;
  output [4:0] if_id_rs;
  output [4:0] if_id_rt;
  output [4:0] id_ex_rd;
  output [4:0] ex_mem_rt;
  
  input pcwrite, if_id_write, mux_select;
  output [4:0] if_id_reg_rs;
  output [4:0] if_id_reg_rt;
  output id_ex_memread;
  
  input [1:0] pcsrc;
  input if_id_reg_flush;
  output equal;
  
  wire [31:0] new_pc;
  wire [31:0] pc;
  wire [31:0] add_result;
  wire [31:0] instruction;
  
  wire [31:0] add_result_reg;
  wire [31:0] read_data_1;
  wire [31:0] read_data_2;
  wire [31:0] equal_input_1;
  wire [31:0] equal_input_2;
  wire [31:0] sign_extend;
  
  wire regwrite_reg, memtoreg_reg, memread_reg, memwrite_reg, alusrc_reg, regdst_reg;
  wire [2:0] aluop_reg;
  wire [31:0] read_data_1_reg;
  wire [31:0] read_data_2_reg;
  wire [31:0] first_input_alu;
  wire [31:0] second_input_alu;
  wire [31:0] sign_extend_reg;
  wire [4:0] instruction_20_16_reg;
  wire [4:0] instruction_15_11_reg;
  wire [4:0] instruction_25_21_reg;
  wire [31:0] shifted;
  wire [31:0] alu_second_input;
  wire [31:0] alu_result;
  wire zero;
  wire [4:0] write_register_new;
  
  wire regwrite_reg_new, memtoreg_reg_new, memread_reg_new, memwrite_reg_new;
  wire [31:0] branch_dst;
  wire [31:0] alu_result_reg;
  wire [31:0] read_data_2_reg_new;
  wire [4:0]  write_register_reg;
  wire [4:0] write_register;
  wire [31:0] read_data;
  
  wire regwrite_reg_new_new, memtoreg_reg_new_new;
  wire [31:0] read_data_reg;
  wire [31:0] alu_result_reg_new;
  
  wire [31:0] write_data;
  
  wire regwrite_mux, memtoreg_mux, memread_mux, memwrite_mux, alusrc_mux, regdst_mux;
  wire [2:0] alu_op_mux;
  
  wire [31:0] sign_extend_jump;
  wire [31:0] jump;
  wire [31:0] jump_dst;
  
  three_input_multiplexer_32_bits MUX0(add_result, branch_dst, jump_dst, pcsrc, new_pc);
  adder ADDER1 (32'b00000000000000000000000000000100, pc, add_result);
  pc_register PC (clk, rst, new_pc, pcwrite, pc); //added enable signal
  instruction_memory IM (clk, rst, pc, instruction);
  
  register_32_bits_en_and_clr AR1 (clk, rst, if_id_reg_flush, add_result, if_id_write, add_result_reg); //added enable and clear signal
  register_32_bits_en_and_clr IR (clk, rst, if_id_reg_flush, instruction, if_id_write, instruction_reg); //added enable and clear signal
  //IF/ID
  register_file RF (clk, rst, instruction_reg[25:21], instruction_reg[20:16], write_register, write_data, regwrite_reg_new_new, read_data_1, read_data_2);
  
  four_input_multiplexer_32_bits MUX4(read_data_1, alu_result, read_data, alu_result_reg,forward_c, equal_input_1);
  four_input_multiplexer_32_bits MUX5(read_data_2, alu_result, read_data, alu_result_reg, forward_d, equal_input_2);
  assign equal = (equal_input_1 == equal_input_2) ? 1'b1 : 1'b0;  //check if rs == rt for beq and bne

  sign_extender SE (instruction_reg[15:0], sign_extend);
  shift_left_two_bits SH2 (sign_extend, shifted); // moved
  adder ADDER2 (add_result_reg, shifted, branch_dst);  
  //jump
  sign_extender SE_jump (instruction_reg[25:0], sign_extend_jump);
  shift_left_two_bits SH2_jump (sign_extend_jump, jump);
  assign jump_dst = {instruction_reg[31:28], jump};
  //
  register_32_bits RD11 (clk, rst, read_data_1, read_data_1_reg);
  register_32_bits RD21 (clk, rst, read_data_2, read_data_2_reg);
  register_32_bits SER (clk, rst, sign_extend, sign_extend_reg);
  //control signals
  two_input_multiplexer_1_bit regwrite_0 (1'b0, regwrite, mux_select, regwrite_mux);
  register_1_bit RW1(clk, rst, regwrite_mux, regwrite_reg);
  
  two_input_multiplexer_1_bit memtoreg_mux_0 (1'b0, memtoreg, mux_select, memtoreg_mux);
  register_1_bit MTR1 (clk, rst, memtoreg_mux, memtoreg_reg);
  
  two_input_multiplexer_1_bit memread_mux_0 (1'b0, memread, mux_select, memread_mux);
  register_1_bit MR1 (clk, rst, memread_mux, memread_reg);
  
  two_input_multiplexer_1_bit memwrite_mux_0 (1'b0, memwrite, mux_select, memwrite_mux);
  register_1_bit MW1 (clk, rst, memwrite_mux, memwrite_reg);
  
  two_input_multiplexer_1_bit alusrc_mux_0 (1'b0, alusrc, mux_select, alusrc_mux);
  register_1_bit ALUSRC1 (clk, rst, alusrc_mux, alusrc_reg);
  
  two_input_multiplexer_3_bits alu_op_mux_0 (3'b0, alu_op, mux_select, alu_op_mux);
  register_3_bits ALUOP1 (clk, rst, alu_op_mux, aluop_reg);
  
  two_input_multiplexer_1_bit regdst_mux_0 (1'b0, regdst, mux_select, regdst_mux);
  register_1_bit RD1 (clk, rst, regdst_mux, regdst_reg);
  //
  register_5_bits OP11 (clk, rst, instruction_reg[25:21], instruction_25_21_reg);
  register_5_bits OP21 (clk, rst, instruction_reg[20:16], instruction_20_16_reg);
  register_5_bits OP31 (clk, rst, instruction_reg[15:11], instruction_15_11_reg);
  //ID/EX
  //control signals:
    //WB (regwrite (1 bit), memtoreg (1 bit)), 
    //MEM (memread (1 bit), memwrite (1 bit)),
    //EX (alusrc (1 bit), aluop (3 bits), regdst (1 bit))
  
  two_input_multiplexer_32_bits MUX1 (read_data_2_reg, sign_extend_reg, alusrc_reg, alu_second_input);
  // two muxes for data hazard
  three_input_multiplexer_32_bits ForwardA (read_data_1_reg, write_data, alu_result_reg, forward_a, first_input_alu);
  three_input_multiplexer_32_bits ForwardB (alu_second_input, write_data, alu_result_reg, forward_b, second_input_alu);
  //
  alu ALU (first_input_alu, second_input_alu, aluop_reg,zero, alu_result);
  two_input_multiplexer_5_bits MUX2 (instruction_20_16_reg, instruction_15_11_reg, regdst_reg, write_register_new);
  //control signals
  register_1_bit RW2(clk, rst, regwrite_reg, regwrite_reg_new);
  register_1_bit MTR2 (clk, rst, memtoreg_reg, memtoreg_reg_new);
  register_1_bit MR2 (clk, rst, memread_reg, memread_reg_new);
  register_1_bit MW2 (clk, rst, memwrite_reg, memwrite_reg_new);
  register_32_bits ALUR1 (clk, rst, alu_result, alu_result_reg);
  register_32_bits RD22 (clk, rst, read_data_2_reg, read_data_2_reg_new);
  register_5_bits WR1 (clk, rst, write_register_new, write_register_reg);
  //EX/MEM
  data_memory DM (clk, rst, alu_result_reg, read_data_2_reg_new, memwrite_reg_new, memread_reg_new, read_data);
  register_5_bits WR2 (clk, rst, write_register_reg, write_register);
  //control signals
  register_1_bit RW3(clk, rst, regwrite_reg_new, regwrite_reg_new_new);
  register_1_bit MTR3 (clk, rst, memtoreg_reg_new, memtoreg_reg_new_new);
  //
  register_32_bits RD (clk, rst, read_data, read_data_reg);
  register_32_bits ALUR2 (clk, rst, alu_result_reg, alu_result_reg_new);
  //MEM/WB  
  two_input_multiplexer_32_bits MUX3 (alu_result_reg_new, read_data_reg, memtoreg_reg_new_new, write_data);
  // outputs for forwarding unit
  assign id_ex_reg_rt = instruction_20_16_reg;
  assign id_ex_reg_rs = instruction_25_21_reg;
  assign ex_mem_reg_rd = write_register_reg;
  assign mem_wb_reg_rd = write_register;
  assign ex_mem_regwrite = regwrite_reg_new;
  assign mem_wb_regwrite = regwrite_reg_new_new;
  //outputs for hazard unit
  assign if_id_reg_rs = instruction_reg[25:21];
  assign if_id_reg_rt = instruction_reg[20:16];
  assign id_ex_memread = memread_reg;
  //branch forwarding unit
  assign ex_mem_memread = memread_reg_new;
  assign id_ex_regwrite = regwrite_reg;
  assign if_id_rs = instruction_reg[25:21];
  assign if_id_rt = instruction_reg[20:16];
  assign id_ex_rd = instruction_15_11_reg;
  assign ex_mem_rt = write_register_reg;
endmodule

//modules

module two_input_multiplexer_32_bits (first_input, second_input, select_signal, out);
  input [31:0] first_input;
  input [31:0] second_input;
  input select_signal;
  output reg [31:0] out;
  always @ (first_input, second_input, select_signal) begin
    out = 32'b0;
    case (select_signal)
      1'b0: out = first_input;
      1'b1: out = second_input;
    endcase
  end
endmodule

module adder (first_input, second_input, out);
  input [31:0] first_input;
  input [31:0] second_input;
  output reg [31:0] out;
  always @ (first_input, second_input) begin
    out = 32'b0;
    out = first_input + second_input;
  end
endmodule

module pc_register (clk, rst, new_pc, pcwrite, pc);
  input clk, rst;
  input [31:0] new_pc;
  input pcwrite;
  output reg [31:0] pc;
  always @ (posedge clk, posedge rst) begin
    if (rst) pc <= 32'b0;
    else if (pcwrite) pc <= new_pc;
  end
endmodule

module instruction_memory (clk, rst, read_address, instruction);
  input clk, rst;
  input [31:0] read_address;
  output [31:0] instruction;
  reg [7:0] instructions[0:511];
  assign instruction = {instructions[read_address + 2'b00], instructions[read_address + 2'b01], instructions[read_address + 2'b10], instructions[read_address + 2'b11]};
  always @ (posedge clk, posedge rst) begin
    if (rst) $readmemb("instructions.txt", instructions);
  end
endmodule

//

module register_file (clk, rst, read_register_1, read_register_2, write_register, write_data, regwrite, read_data_1, read_data_2);
  input clk, rst;
  input [4:0] read_register_1;
  input [4:0] read_register_2;
  input [4:0] write_register;
  input [31:0] write_data;
  input regwrite;
  output [31:0] read_data_1;
  output [31:0] read_data_2;
  reg [31:0] register[0:31];
  assign read_data_1 = register[read_register_1];
  assign read_data_2 = register[read_register_2];
  
  always @ (posedge rst) begin
    if (rst) register[0] <= 32'b0;
  end
  
  always @ (negedge clk) begin
      if (regwrite) register[write_register] <= write_data;
  end
endmodule

module sign_extender (in, out);
  input [15:0] in;
  output [31:0] out;
  assign out = {{16{in[15]}}, in[15:0]};
endmodule

//

module alu (first_input, second_input, alu_op, zero, alu_result);
  input [31:0] first_input;
  input [31:0] second_input;
  input [2:0] alu_op;
  output zero;
  output reg [31:0] alu_result;
  always @ (first_input, second_input, alu_op) begin
    case (alu_op)
      3'b000: alu_result = first_input & second_input;
      3'b001: alu_result = first_input | second_input;
      3'b010: alu_result = first_input + second_input;
      3'b110: alu_result = first_input - second_input;
      3'b111: alu_result = (first_input < second_input);
      default: alu_result = first_input < second_input;
    endcase
  end
  assign zero = (alu_result == 32'b0) ? 1'b1 : 1'b0;
endmodule

module shift_left_two_bits (in, out);
  input [31:0] in;
  output [31:0] out;
  assign out = {in[29:0], 2'b00};
endmodule

module two_input_multiplexer_5_bits (first_input, second_input, select_signal, out);
  input [4:0] first_input;
  input [4:0] second_input;
  input select_signal;
  output reg [4:0] out;
  always @ (first_input, second_input, select_signal) begin
    out = 4'b0;
    case (select_signal)
      1'b0: out = first_input;
      1'b1: out = second_input;
    endcase
  end
endmodule

//

module data_memory (clk, rst, address, write_data, memwrite, memread, read_data);
  input clk, rst;
  input [31:0] address;
  input [31:0] write_data;
  input memwrite;
  input memread;
  output [31:0] read_data;
  reg [31:0] memory[0:2047];
  always @ (posedge clk, posedge rst) begin
    if (rst) $readmemb ("dataMemory.txt", memory);
    else begin
      if (memwrite) memory[address] <= write_data;
    end
  end
  assign read_data = memory[address];
endmodule


//

module register_32_bits (clk, rst, in, out);
  input clk, rst;
  input [31:0] in;
  output reg [31:0] out;
  always @ (posedge clk) begin
    out <= in;
  end
endmodule

module register_1_bit (clk, rst, in, out);
  input clk, rst;
  input in;
  output reg out;
  always @ (posedge clk) begin
    out <= in;
  end
endmodule

module register_3_bits (clk, rst, in, out);
  input clk, rst;
  input [2:0] in;
  output reg [2:0] out;
  always @ (posedge clk) begin
    out <= in;
  end
endmodule

module register_5_bits (clk, rst, in, out);
  input clk, rst;
  input [4:0] in;
  output reg [4:0] out;
  always @ (posedge clk) begin
    out <= in;
  end
endmodule

//

module three_input_multiplexer_32_bits (first_input, second_input, third_input, select_signal, out);
  input [31:0] first_input;
  input [31:0] second_input;
  input [31:0] third_input;
  input [1:0] select_signal;
  output reg [31:0] out;
  always @ (first_input, second_input, third_input, select_signal) begin
    case (select_signal)
      2'b00: out = first_input;
      2'b01: out = second_input;
      2'b10: out = third_input;
    endcase
  end
endmodule

//
module four_input_multiplexer_32_bits (first_input, second_input, third_input, fourth_input, select_signal, out);
  input [31:0] first_input;
  input [31:0] second_input;
  input [31:0] third_input;
  input [31:0] fourth_input;
  input [1:0] select_signal;
  output reg [31:0] out;
  always @ (first_input, second_input, third_input, select_signal) begin
    case (select_signal)
      2'b00: out = first_input;
      2'b01: out = second_input;
      2'b10: out = third_input;
      2'b11: out = fourth_input;
    endcase
  end
endmodule

//
module register_32_bits_en (clk, rst, in, en, out);
  input clk, rst;
  input [31:0] in;
  input en;
  output reg [31:0] out;
  always @ (posedge clk) begin
    if (en) out <= in;
  end
endmodule

module register_1_bit_en (clk, rst, in, en, out);
  input clk, rst;
  input in;
  input en;
  output reg out;
  always @ (posedge clk) begin
    if (en) out <= in;
  end
endmodule

module register_3_bits_en (clk, rst, in, en, out);
  input clk, rst;
  input [2:0] in;
  input en;
  output reg [2:0] out;
  always @ (posedge clk) begin
    if (en) out <= in;
  end
endmodule

module register_5_bits_en (clk, rst, in, en, out);
  input clk, rst;
  input [4:0] in;
  input en;
  output reg [4:0] out;
  always @ (posedge clk) begin
    if (en) out <= in;
  end
endmodule

module two_input_multiplexer_3_bits (first_input, second_input, select_signal, out);
  input [2:0] first_input;
  input [2:0] second_input;
  input select_signal;
  output reg [2:0] out;
  always @ (first_input, second_input, select_signal) begin
    out = 1'b0;
    case (select_signal)
      1'b0: out = first_input;
      1'b1: out = second_input;
    endcase
  end
endmodule

module two_input_multiplexer_1_bit (first_input, second_input, select_signal, out);
  input first_input;
  input second_input;
  input select_signal;
  output reg out;
  always @ (first_input, second_input, select_signal) begin
    out = 1'b0;
    case (select_signal)
      1'b0: out = first_input;
      1'b1: out = second_input;
    endcase
  end
endmodule
//
//
module register_32_bits_en_and_clr (clk, rst, clr, in, en, out);
  input clk, rst;
  input [31:0] in;
  input en, clr;
  output reg [31:0] out;
  always @ (posedge clk) begin
    if (clr) out <= 32'b0;
    else if (en) out <= in;
  end
endmodule
//

//IF/ID
// add_result (32 bits), instruction (32 bits)

//ID/EX
/* WB (regwrite (1 bit), memtoreg (1 bit)), 
MEM (memread (1 bit), memwrite (1 bit), pcsrc (1 bit)), 
EX (alusrc (1 bit), aluop (3 bits), regdst (1 bit))
add_result (32 bits), raed_data_1 (32 bits), raed_data_2 (32 bits),
sign_extend (32 bits), instr[20-16] (5 bits), instr[15-11] (5 bits) 
*/

//EX/MEM
/* WB (regwrite (1 bit), memtoreg (1 bit)), 
MEM (memread (1 bit), memwrite (1 bit), pcsrc (1 bit)), 
add_result (32 bits), alu_result (32 bits), zero (1 bit),
raed_data_2 (32 bits), instr[20-16] or instr[15-11] (5 bits) 
*/

//MEM/WB
/* WB (regwrite (1 bit), memtoreg (1 bit)), 
raed_data (32 bits), alu_result (32 bits),
raed_data_2 (32 bits), instr[20-16] or instr[15-11] (5 bits) 
*/

module test();
  reg clk,rst;
  MIPS mips(clk,rst);
  initial begin
    rst = 1'b0;
    #40 rst = 1'b1; #40 rst = 1'b0; #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
    #80 #20 clk = 1'b1;
    #80 #20 clk = 1'b0;
  end
endmodule



