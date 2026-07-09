`timescale 1ns/1ps
module tb_rest_um7;
  reg clk=0; always #5 clk=~clk;
  reg rst=1;
  reg model_start=0;
  wire model_done;
  wire [31:0] roll,pitch,yaw,p,q,r;
  rov_model u_m(.clk(clk),.rst(rst),.start(model_start),.done(model_done),
    .tau0_roll(0),.tau0_pitch(0),.tau0_yaw(0),
    .tau1_roll(0),.tau1_pitch(0),.tau1_yaw(0),
    .state_roll(roll),.state_pitch(pitch),.state_yaw(yaw),
    .state_p(p),.state_q(q),.state_r(r),
    .dbg_st(),.dbg_kphase(),.dbg_rhs0(),.dbg_rhs1(),.dbg_rhs2(),
    .dbg_n0(),.dbg_n1(),.dbg_n2(),.dbg_det(),.dbg_Pd(),.dbg_Qd(),.dbg_Rd());

  reg model_done_d=0;
  always @(posedge clk) model_done_d <= model_done;
  wire sensor_start = model_done & ~model_done_d;
  wire sensor_done;
  wire [31:0] mr,mp,my,mpp,mq,mrr;
  sensor_model #(.PASSTHROUGH(1)) u_s(
    .clk(clk),.rst(rst),.start(sensor_start),.done(sensor_done),
    .roll_in(roll),.pitch_in(pitch),.yaw_in(yaw),.p_in(p),.q_in(q),.r_in(r),
    .roll_out(mr),.pitch_out(mp),.yaw_out(my),.p_out(mpp),.q_out(mq),.r_out(mrr));

  reg [31:0] cr,cp,cy,cpp,cq,crr;
  reg cap_pending=0;
  wire um7_busy, um7_done, send_byte;
  wire [7:0] byte_out;
  wire um7_start = cap_pending & ~um7_busy;
  always @(posedge clk) begin
    if (rst) cap_pending<=0;
    else if (sensor_done) begin
      cr<=mr; cp<=mp; cy<=my; cpp<=mpp; cq<=mq; crr<=mrr; cap_pending<=1;
    end else if (um7_start) cap_pending<=0;
  end
  um7_tx u_u(.clk(clk),.rst(rst),.start(um7_start),.done(um7_done),.busy(um7_busy),
    .roll(cr),.pitch(cp),.yaw(cy),.p(cpp),.q(cq),.r(crr),
    .send_byte(send_byte),.byte_out(byte_out),.byte_busy(1'b0),.byte_done(send_byte));

  integer k, nbyte;
  reg [7:0] bytes[0:40];
  reg [15:0] w16,x16,y16,z16;
  real rr,pp,yy;
  initial begin
    nbyte=0;
    repeat(20) @(posedge clk); rst=0; repeat(5) @(posedge clk);
    for (k=0;k<50;k=k+1) begin
      nbyte=0;
      @(posedge clk); model_start<=1; @(posedge clk); model_start<=0;
      // collect um7 bytes for this step
      while (1) begin
        @(posedge clk);
        if (send_byte) begin bytes[nbyte]=byte_out; nbyte=nbyte+1; end
        if (um7_done) begin
          // wait one more for done
          @(posedge clk);
          disable break_loop;
        end
        if (nbyte>40) disable break_loop;
      end
      begin : break_loop end
      w16={bytes[5],bytes[6]}; x16={bytes[7],bytes[8]};
      y16={bytes[9],bytes[10]}; z16={bytes[11],bytes[12]};
      rr=$itor($signed(roll))/65536.0*57.29578;
      pp=$itor($signed(pitch))/65536.0*57.29578;
      yy=$itor($signed(yaw))/65536.0*57.29578;
      if (k<5 || k==49)
        $display("k=%0d plant_rpy=%8.4f %8.4f %8.4f  quat_i16=%0d %0d %0d %0d  nbytes=%0d",
          k, rr,pp,yy, $signed(w16),$signed(x16),$signed(y16),$signed(z16), nbyte);
    end
    $finish;
  end
endmodule
