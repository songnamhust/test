/* verilator lint_off EOFNEWLINE */
module ex_mem_pipeline_reg(
    IN_INSTRUCTION,  //INSTRUCTION [11:7]
    IN_PC,
    IN_ALU_RESULT, 
    IN_DATA2, 
    IN_IMMEDIATE,
    IN_DATAMEMSEL,
    IN_READ_WRITE,
    IN_WB_SEL,
    IN_REG_WRITE_EN,
    OUT_INSTRUCTION,
    OUT_PC,
    OUT_ALU_RESULT,
    OUT_DATA2,
    OUT_IMMEDIATE, 
    OUT_DATAMEMSEL,
    OUT_READ_WRITE,
    OUT_WB_SEL,
    OUT_REG_WRITE_EN,
    CLK, 
    RST_N);

    input [4:0] IN_INSTRUCTION;
    input [1:0] IN_WB_SEL;
    input [3:0] IN_READ_WRITE;
    input [31:0] IN_PC, IN_ALU_RESULT, IN_DATA2, IN_IMMEDIATE;   
    input IN_DATAMEMSEL, IN_REG_WRITE_EN, CLK, RST_N;

    output reg [4:0] OUT_INSTRUCTION;
    output reg [1:0] OUT_WB_SEL;
    output reg [3:0] OUT_READ_WRITE;
    output reg [31:0] OUT_PC, OUT_ALU_RESULT, OUT_DATA2, OUT_IMMEDIATE; 
    output reg OUT_DATAMEMSEL, OUT_REG_WRITE_EN;

    always @(posedge CLK or negedge RST_N) begin
        if (!RST_N) begin
            OUT_INSTRUCTION <= 5'b0;
            OUT_PC <= 32'b0;
            OUT_ALU_RESULT <= 32'b0;
            OUT_DATA2 <= 32'b0;
            OUT_IMMEDIATE <= 32'b0;
            OUT_DATAMEMSEL <= 1'bx;
            OUT_READ_WRITE <= 4'bx;
            OUT_WB_SEL <= 2'bx;
            OUT_REG_WRITE_EN <= 1'bx;
        end
    else begin
            OUT_INSTRUCTION <= IN_INSTRUCTION;
            OUT_PC <= IN_PC;
            OUT_ALU_RESULT <= IN_ALU_RESULT;
            OUT_DATA2 <= IN_DATA2;
            OUT_IMMEDIATE <= IN_IMMEDIATE;
            OUT_DATAMEMSEL <= IN_DATAMEMSEL;
            OUT_READ_WRITE <= IN_READ_WRITE;
            OUT_WB_SEL <= IN_WB_SEL;
            OUT_REG_WRITE_EN <= IN_REG_WRITE_EN;
        end
    end

endmodule

 /* verilator lint_on EOFNEWLINE */
