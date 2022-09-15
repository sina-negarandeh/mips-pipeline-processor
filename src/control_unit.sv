//MemToReg = DataRegSelect

module ALUControl (ALUOp, func, ALUOperation);
  input [1:0]ALUOp;
  input [5:0]func;
  output reg [2:0]ALUOperation;
  always @(ALUOp or func) begin
    case (ALUOp)
      2'b00: ALUOperation = 3'b010;  //OPC: Lw and Sw and Addi
                                  //ALU Operation: Add
      2'b01: ALUOperation = 3'b110;  //OPC: Beq
                                  //ALU Operation: Sub
      2'b10: case (func) //R-Type
            6'b100000: ALUOperation = 3'b010; //Add
            6'b100010: ALUOperation = 3'b110; //Sub
            6'b100100: ALUOperation = 3'b000; //And
            6'b100101: ALUOperation = 3'b001; //Or
            6'b101010: ALUOperation = 3'b111; //Slt
            default: ALUOperation = 3'b010;
          endcase
      2'b11: ALUOperation = 3'b000; //OPC: Andi
                                 //ALU Operation: And
      default: ALUOperation = 3'b010;
    endcase
  end
endmodule

module Control (OPC, RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, ALUOp);
  input [5:0]OPC;
  output reg [1:0]RegDst;
  output reg ALUSrc;
  output reg MemToReg;
  output reg RegWrite, MemRead, MemWrite;
  output reg [1:0]ALUOp;
  always @(OPC) begin
    case (OPC)
      6'b000000: begin //R-Type
        RegDst = 1'b1;
        ALUSrc = 1'b0;
        MemToReg = 1'b0;
        RegWrite = 1'b1;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUOp = 2'b10;
      end
      6'b000010: begin //J
        RegWrite = 1'b0;
        MemRead = 2'b0;
        MemWrite = 1'b0;
      end
      6'b001000: begin //Addi
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        MemToReg = 1'b0;
        RegWrite = 1'b1;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUOp = 2'b00;
      end
      6'b001100: begin //Andi
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        MemToReg = 1'b0;
        RegWrite = 1'b1;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUOp = 2'b11;
      end
      6'b100011: begin //Lw
        RegDst = 2'b0;
        ALUSrc = 1'b1;
        MemToReg = 1'b1;
        RegWrite = 1'b1;
        MemRead = 1'b1;
        MemWrite = 1'b0;
        ALUOp = 2'b00;
      end
      6'b101011: begin //Sw
        ALUSrc = 1'b1;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b1;
        ALUOp = 2'b00;
      end
      6'b000100: begin //Beq
        ALUSrc = 1'b0;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUOp = 2'b01;
      end
      6'b000101: begin //Bne
        ALUSrc = 1'b0;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUOp = 2'b01;
      end
    endcase
  end
endmodule

module forwarding_unit (id_ex_reg_rt, id_ex_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, forward_a, forward_b);
  input [4:0] id_ex_reg_rt;
  input [4:0] id_ex_reg_rs;
  input [4:0] ex_mem_reg_rd;
  input [4:0] mem_wb_reg_rd;
  input ex_mem_regwrite, mem_wb_regwrite;
  
  output reg [1:0] forward_a;
  output reg [1:0] forward_b;
  
  always @ (id_ex_reg_rt, id_ex_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite) begin
    //without data hazard
    forward_a = 2'b00;
    forward_b = 2'b00;
    //EX/MEM data hazard
    if ((ex_mem_regwrite) && (ex_mem_reg_rd == id_ex_reg_rs))
      forward_a = 2'b10;  //first alu source
    if ((ex_mem_regwrite) && (ex_mem_reg_rd == id_ex_reg_rt))
      forward_b = 2'b10;  //second alu source
    //MEM/WB data hazard
    if (((mem_wb_regwrite) && (mem_wb_reg_rd == id_ex_reg_rs)) && ((ex_mem_reg_rd != id_ex_reg_rs) || (ex_mem_regwrite == 1'b0)))
      forward_a = 2'b01;  //first alu source
    if (((mem_wb_regwrite) && (mem_wb_reg_rd == id_ex_reg_rt)) && ((ex_mem_reg_rd != id_ex_reg_rt) || (ex_mem_regwrite == 1'b0)))
      forward_b = 2'b01;  //second alu source
  end
endmodule

module branch_forwarding_unit(if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, ex_mem_regwrite, forward_c, forward_d);
  input ex_mem_memread;
  input id_ex_regwrite;
  input [4:0] if_id_rs;
  input [4:0] if_id_rt;
  input [4:0] id_ex_rd;
  input [4:0] ex_mem_rt;
  input ex_mem_regwrite;
  
  output reg[1:0] forward_c;
  output reg[1:0] forward_d;
  
  //rt - beq/bne and lw - beq/bne
  always @(if_id_rs, if_id_rt, id_ex_rd, id_ex_regwrite) begin
    if (id_ex_regwrite && id_ex_rd == if_id_rs) forward_c = 2'b01;
    else if (ex_mem_memread && ex_mem_rt == if_id_rs) forward_c = 2'b10;
    else if (ex_mem_regwrite && ex_mem_rt == if_id_rs) forward_c = 2'b11;
    else forward_c = 2'b00;
      
    if (id_ex_regwrite && id_ex_rd == if_id_rt) forward_d = 2'b01;
    else if (ex_mem_memread && ex_mem_rt == if_id_rt) forward_d = 2'b10;
    else if (ex_mem_regwrite && ex_mem_rt == if_id_rt) forward_d = 2'b11;
    else forward_d = 2'b00;
  end
endmodule

module hazard_unit (rst, if_id_reg_rs, if_id_reg_rt, id_ex_memread, id_ex_reg_rt, pcwrite, if_id_write, mux_select);
  input [4:0] if_id_reg_rs;
  input [4:0] if_id_reg_rt;
  input id_ex_memread,rst;
  input [4:0] id_ex_reg_rt;
  
  output reg pcwrite;
  output reg if_id_write;
  output reg mux_select;
  
  always @ (posedge rst) begin
    if (rst == 1'b1) begin
      pcwrite = 1'b1;
      if_id_write = 1'b1;
      mux_select = 1'b1;
    end
  end
  
  always @ (if_id_reg_rs, if_id_reg_rt, id_ex_memread, id_ex_reg_rt) begin
    if ((id_ex_memread) && ((id_ex_reg_rt == if_id_reg_rs) || (id_ex_reg_rt == if_id_reg_rt))) begin //lw
      pcwrite = 1'b0; //fetching again
      if_id_write = 1'b0; //decoding again 
      mux_select = 1'b0;  //bubble
    end
    else begin
      pcwrite = 1'b1;
      if_id_write = 1'b1;
      mux_select = 1'b1;
    end
  end
endmodule


module ControlUnit (rst,OPC, func, id_ex_reg_rt, id_ex_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, if_id_reg_rs, if_id_reg_rt, id_ex_memread, RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, ALUOperation, forward_a, forward_b, pcwrite, if_id_write, mux_select, equal, pcsrc, if_id_reg_flush,if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, forward_c, forward_d);
  input rst;
  input [5:0]OPC;
  input [5:0]func;
  output [1:0]RegDst;
  output ALUSrc;
  output MemToReg;
  output RegWrite, MemRead, MemWrite;
  output [2:0]ALUOperation;
  wire [1:0]ALUOp;
  
  input [4:0] id_ex_reg_rt;
  input [4:0] id_ex_reg_rs;
  input [4:0] ex_mem_reg_rd;
  input [4:0] mem_wb_reg_rd;
  input ex_mem_regwrite, mem_wb_regwrite;
  output [1:0] forward_a;
  output [1:0] forward_b;
  output [1:0] forward_c;
  output [1:0] forward_d;
  
  input [4:0] if_id_reg_rs;
  input [4:0] if_id_reg_rt;
  input id_ex_memread;
  
  input ex_mem_memread;
  input id_ex_regwrite;
  input [4:0] if_id_rs;
  input [4:0] if_id_rt;
  input [4:0] id_ex_rd;
  input [4:0] ex_mem_rt;
  
  output pcwrite;
  output if_id_write;
  output mux_select;
  
  input equal;
  output reg [1:0] pcsrc;
  output reg if_id_reg_flush;
  
  Control C0 (OPC, RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, ALUOp);
  ALUControl C1 (ALUOp, func, ALUOperation);
  forwarding_unit ForwardingUnit (id_ex_reg_rt, id_ex_reg_rs, ex_mem_reg_rd, mem_wb_reg_rd, ex_mem_regwrite, mem_wb_regwrite, forward_a, forward_b);
  branch_forwarding_unit BranchForwardingUnit(if_id_rs, if_id_rt ,id_ex_rd, id_ex_regwrite, ex_mem_memread, ex_mem_rt, ex_mem_regwrite, forward_c, forward_d);
  hazard_unit HazardUnit (rst,if_id_reg_rs, if_id_reg_rt, id_ex_memread, id_ex_reg_rt, pcwrite, if_id_write, mux_select);
  
  always @ (posedge rst) begin
    if (rst == 1'b1) pcsrc = 2'b00;
  end
  
  always @ (OPC, func, equal) begin
    if (OPC == 6'b000010) begin //jump
      pcsrc = 2'b10;
      if_id_reg_flush = 1'b1;
    end
    else if (OPC == 6'b000100 && equal == 1'b1) begin //beq
      pcsrc = 2'b01;
      if_id_reg_flush = 1'b1;
    end
    else if (OPC == 6'b000101 && equal == 1'b0) begin //bne
      pcsrc = 2'b01;
      if_id_reg_flush = 1'b1;
    end
    else begin
      pcsrc = 1'b00; //non control flow commmand
      if_id_reg_flush = 1'b0;
    end
  end 
endmodule
