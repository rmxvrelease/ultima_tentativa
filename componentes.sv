module adder(
  input logic [31:0] a,
  input logic [31:0] b,
  output logic [31:0] o
);
  assign o = a + b;
endmodule

module alu (
    input logic [31:0] a,
    input logic [31:0] b,
    input logic [4:0] op,
    
    output logic [31:0] o,
    output logic zero
);

    localparam OP_ADD = 3'b000;
    localparam OP_SUB = 3'b001;
    localparam OP_AND = 3'b010;
    localparam OP_OR = 3'b011;
    localparam OP_SLT = 3'b100;

    always_comb begin
        o = 32'b0;

        case(op)
            OP_ADD: begin
                o <= a + b;
            end

            OP_SUB: begin
                o <= a - b;
            end

            OP_AND: begin
                o <= a & b;
            end

            OP_OR: begin
                o <= a | b;
            end

            OP_SLT: begin
                if($signed(a) < $signed(b)) begin
                    o <= 32'd1;
                end else begin
                    o <= 32'd0;
                end
            end

            default: begin
                o <= 32'd0;
            end
        endcase
    end

    assign zero = (o==32'd0);

endmodule


module registers(
  input logic clk,
  input logic enable_write,
  input logic [31:0] write_data,
  input logic [4:0] rs1,
  input logic [4:0] rs2,
  input logic [4:0] rd,
  output logic [31:0] data_1,
  output logic [31:0] data_2,
  output logic [31:0] reg0_window,
  output logic [31:0] reg1_window,
  output logic [31:0] reg2_window,
  output logic [31:0] reg3_window
);
  reg [31:0] regs [31:0];
  always @(posedge clk) begin
    if (enable_write && rd != 0) begin
      regs[rd] <= write_data;
    end
  end 
  assign data_1 = (rs1 == 0) ? 32'b0 : regs[rs1];
  assign data_2 = (rs2 == 0) ? 32'b0 : regs[rs2];
  assign reg0_window = regs[5'b00000];
  assign reg1_window = regs[5'b00001];
  assign reg2_window = regs[5'b00010];
  assign reg3_window = regs[5'b00011];
endmodule


module mux32_1(
  input logic [31:0] a,
  input logic [31:0] b,
  input logic c,
  output logic [31:0] o
);
  always_comb begin
    case (c)
      1'b0: begin o <= a; end 
      default: begin o <= b; end
    endcase
  end
endmodule


module control_unit(
	input logic [31:0] instruction,
	output logic mem_to_reg,
	output logic read_from_mem,
	output logic write_to_mem, 
	output logic branch,
	output logic [4:0] alu_op,
	output logic alu_origin,
	output logic write_to_reg
);
	always_comb begin
    case (instruction[6:0])
    7'b0110011: begin
      // add, and, sub, or, slt
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b0;
      case (instruction[31:25])
      7'b0100000: begin
        alu_op <= 5'b00001;
      end
      default: begin
        case (instruction[14:12])
          3'b000: begin
            alu_op <= 5'b00000;
          end
          3'b010: begin
            alu_op <= 5'b00100;
          end
          3'b110: begin
            alu_op <= 5'b00011;
          end
          default: begin
            alu_op <= 5'b00010;
          end
        endcase
        end
      endcase
    end
    7'b0000011: begin
      // lw
      mem_to_reg <= 1'b1;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
    end
    7'b0100011: begin
      // sw
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b1;
      write_to_reg <= 1'b0;
      alu_origin <= 1'b1;
    end
    7'b1100011: begin
      // beq
      mem_to_reg <= 1'b0;
      branch <= 1'b1;
      alu_op <= 5'b00001;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b0;
      alu_origin <= 1'b0;
    end
    7'b0010011: begin
      //addi
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
    end
    7'b1101111: begin
      // jal (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
    end
    7'b1100111: begin
      // jalr (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
    end
    default: begin
      // lui (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
    end
    endcase
	end
  assign read_from_mem = 1'b1;
endmodule

module immediate_generator(
    input  logic [31:0] instr,
    output logic [31:0] imm
); 
always_comb begin
    case (instr[6:0])
        7'b1101111: begin  // jal
            imm = { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 };
        end

        7'b0100011: begin  // sw
            imm = { {20{instr[31]}}, instr[31:25], instr[11:7] };
        end

        7'b1100011: begin  // beq
            imm = { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
        end

        7'b0110111: begin  // lui
            imm = {instr[31:12], 12'b0};
        end

        // Tipo-I
        7'b0000011,  // lw
        7'b1100111,  // jalr
        7'b0010011:  // addi
        begin
            imm = { {20{instr[31]}}, instr[31:20] };
        end

        default: begin
            imm = 32'b0;
        end
    endcase
end
endmodule

module plus_four(
  input logic [31:0] a,
  output logic [31:0] o
);
  assign o = a + 32'b0100;
endmodule

module program_counter(
  input logic clk,
  input logic [31:0] in,
  output logic [31:0] out
);
  reg [31:0] pc_value;
  always @(posedge clk) begin
    pc_value <= in;
  end
  assign out = pc_value;
endmodule

module programa_1(
  input logic clk,
  input logic [31:0] add,
  output logic [31:0] ins
);
  always @(posedge clk) begin
    case (add)
    32'b00000: ins = 32'b00000000000100001000000010010011;
    32'b00100: ins = 32'b00000000000100010000000100010011;
    32'b01000: ins = 32'b00000000001000001000000110110011;
    default: ins = 32'b0;
    endcase
  end
endmodule
