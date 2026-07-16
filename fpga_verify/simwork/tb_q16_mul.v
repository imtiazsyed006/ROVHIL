`timescale 1ns/1ps
module tb_q16_mul;
  reg clk=0; always #5 clk=~clk;
  reg rst=1, start=0;
  reg signed [31:0] a=0, b=0;
  wire signed [31:0] p;
  wire valid, busy;
  q16_mul u(.clk(clk),.rst(rst),.start(start),.a(a),.b(b),.p(p),.valid(valid),.busy(busy));

  function signed [31:0] fmul;
    input signed [31:0] x,y;
    reg signed [63:0] pr;
    begin pr = x*y + 64'sd32768; fmul = pr[47:16]; end
  endfunction

  integer i, errs;
  reg signed [31:0] aa, bb, exp;
  initial begin
    errs=0;
    repeat(10) @(posedge clk); rst=0; repeat(5) @(posedge clk);
    // identity * 1.0
    a=32'sd65536; b=32'sd65536; // 1*1
    @(posedge clk); start<=1; @(posedge clk); start<=0;
    while(!valid) @(posedge clk);
    exp = fmul(32'sd65536,32'sd65536);
    $display("1*1 got=%0d exp=%0d", p, exp);
    if (p!==exp) errs=errs+1;

    a=-32'sd65536; b=32'sd131072; // -1 * 2
    @(posedge clk); start<=1; @(posedge clk); start<=0;
    while(!valid) @(posedge clk);
    exp = fmul(-32'sd65536,32'sd131072);
    $display("-1*2 got=%0d exp=%0d", p, exp);
    if (p!==exp) errs=errs+1;

    for (i=0;i<200;i=i+1) begin
      aa = $random; bb = $random;
      // keep moderate magnitude
      aa = aa >>> 8; bb = bb >>> 8;
      a=aa; b=bb;
      @(posedge clk); start<=1; @(posedge clk); start<=0;
      while(!valid) @(posedge clk);
      exp = fmul(aa,bb);
      if (p !== exp) begin
        errs=errs+1;
        if (errs<=5) $display("MISMATCH i=%0d a=%0d b=%0d got=%0d exp=%0d", i,aa,bb,p,exp);
      end
    end
    $display("errs=%0d", errs);
    if (errs!=0) $finish(1);
    $display("PASS q16_mul");
    $finish;
  end
endmodule
