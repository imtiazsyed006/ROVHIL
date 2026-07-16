`timescale 1ns/1ps
module tb_sensor_noise;
  reg clk=0; always #5 clk=~clk;
  reg rst=1, start=0; wire done;
  reg signed [31:0] ri=0,pi=0,yi=0,p=0,q=0,r=0;
  wire signed [31:0] ro,po,yo,op,oq,orr;
  sensor_model #(.PASSTHROUGH(0)) u(.clk(clk),.rst(rst),.start(start),.done(done),.roll_in(ri),.pitch_in(pi),.yaw_in(yi),.p_in(p),.q_in(q),.r_in(r),.roll_out(ro),.pitch_out(po),.yaw_out(yo),.p_out(op),.q_out(oq),.r_out(orr));
  initial begin
    repeat(20) @(posedge clk); rst=0; repeat(5) @(posedge clk);
    ri=0;pi=0;yi=0; @(posedge clk); start<=1; @(posedge clk); start<=0; while(!done) @(posedge clk);
    $display("rest deg RPY %.3f %.3f %.3f", $itor(ro)/65536.0*57.3, $itor(po)/65536.0*57.3, $itor(yo)/65536.0*57.3);
    ri=32'sd6554; pi=-32'sd3277; yi=32'sd13107; p=32'sd32768; q=-32'sd19661; r=32'sd6554;
    @(posedge clk); start<=1; @(posedge clk); start<=0; while(!done) @(posedge clk);
    $display("mot  deg RPY %.3f %.3f %.3f", $itor(ro)/65536.0*57.3, $itor(po)/65536.0*57.3, $itor(yo)/65536.0*57.3);
    $display("mot  dps pqr %.3f %.3f %.3f", $itor(op)/65536.0*57.3, $itor(oq)/65536.0*57.3, $itor(orr)/65536.0*57.3);
    $finish;
  end
endmodule
