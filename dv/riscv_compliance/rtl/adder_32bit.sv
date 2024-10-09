/* verilator lint_off EOFNEWLINE */
module adder_32bit (IN, OUT);

    input [31:0] IN;
    output [31:0] OUT;

    assign OUT = IN + 32'd4;

endmodule

/* verilator lint_on EOFNEWLINE */
