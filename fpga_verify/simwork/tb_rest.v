`timescale 1ns/1ps
module tb_rest;
  reg clk=0; always #5 clk=~clk;
  reg rst=1, start=0;
  wire done;
  wire [31:0] roll,pitch,yaw,p,q,r;
  rov_model u(.clk(clk),.rst(rst),.start(start),.done(done),
    .tau0_roll(0),.tau0_pitch(0),.tau0_yaw(0),
    .tau1_roll(0),.tau1_pitch(0),.tau1_yaw(0),
    .state_roll(roll),.state_pitch(pitch),.state_yaw(yaw),
    .state_p(p),.state_q(q),.state_r(r),
    .dbg_st(),.dbg_kphase(),.dbg_rhs0(),.dbg_rhs1(),.dbg_rhs2(),
    .dbg_n0(),.dbg_n1(),.dbg_n2(),.dbg_det(),.dbg_Pd(),.dbg_Qd(),.dbg_Rd());
  integer k;
  real rr,pp,yy;
  initial begin
    repeat(20) @(posedge clk); rst=0; repeat(5) @(posedge clk);
    for (k=0;k<100;k=k+1) begin
      @(posedge clk); start<=1; @(posedge clk); start<=0;
      while(!done) @(posedge clk);
      if (k<5 || k==99) begin
        rr=$itor($signed(roll))/65536.0*57.29578;
        pp=$itor($signed(pitch))/65536.0*57.29578;
        yy=$itor($signed(yaw))/65536.0*57.29578;
        $display("k=%0d rpy_deg=%f %f %f pqr=%f %f %f", k, rr,pp,yy,
          $itor($signed(p))/65536.0*57.29578,
          $itor($signed(q))/65536.0*57.29578,
          $itor($signed(r))/65536.0*57.29578);
      end
    end
    $finish;
  end
endmodule
