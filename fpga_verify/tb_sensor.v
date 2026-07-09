`timescale 1ns / 1ps
//----------------------------------------------------------------------
// tb_sensor.v -- Smoke test for sensor_model.v
// Feeds a few known (RPY, pqr) samples and prints measured outputs.
//----------------------------------------------------------------------
module tb_sensor;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg start = 0;
    wire done;

    reg  signed [31:0] roll_in, pitch_in, yaw_in, p_in, q_in, r_in;
    wire signed [31:0] roll_out, pitch_out, yaw_out, p_out, q_out, r_out;

    sensor_model u_dut (
        .clk(clk), .rst(rst), .start(start), .done(done),
        .roll_in(roll_in), .pitch_in(pitch_in), .yaw_in(yaw_in),
        .p_in(p_in), .q_in(q_in), .r_in(r_in),
        .roll_out(roll_out), .pitch_out(pitch_out), .yaw_out(yaw_out),
        .p_out(p_out), .q_out(q_out), .r_out(r_out)
    );

    // float ≈ Q16.16 / 65536
    function real q2f;
        input signed [31:0] v;
        begin q2f = $itor(v) / 65536.0; end
    endfunction

    integer k;
    initial begin
        roll_in = 0; pitch_in = 0; yaw_in = 0;
        p_in = 0; q_in = 0; r_in = 0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // sample 0: level, zero rate  → should show biases
        roll_in  = 32'sd0;
        pitch_in = 32'sd0;
        yaw_in   = 32'sd0;
        p_in = 0; q_in = 0; r_in = 0;
        @(posedge clk); start <= 1;
        @(posedge clk); start <= 0;
        while (!done) @(posedge clk);
        $display("s0 att_deg  = %10.4f %10.4f %10.4f",
                 q2f(roll_out)*57.29578, q2f(pitch_out)*57.29578, q2f(yaw_out)*57.29578);
        $display("s0 rate_dps = %10.4f %10.4f %10.4f",
                 q2f(p_out)*57.29578, q2f(q_out)*57.29578, q2f(r_out)*57.29578);

        // sample 1: non-zero motion
        roll_in  = 32'sd6554;    // 0.1 rad
        pitch_in = -32'sd3277;   // -0.05 rad
        yaw_in   = 32'sd13107;   // 0.2 rad
        p_in     = 32'sd32768;   // 0.5 rad/s
        q_in     = -32'sd19661;  // -0.3 rad/s
        r_in     = 32'sd6554;    // 0.1 rad/s
        @(posedge clk); start <= 1;
        @(posedge clk); start <= 0;
        while (!done) @(posedge clk);
        $display("s1 att_deg  = %10.4f %10.4f %10.4f",
                 q2f(roll_out)*57.29578, q2f(pitch_out)*57.29578, q2f(yaw_out)*57.29578);
        $display("s1 rate_dps = %10.4f %10.4f %10.4f",
                 q2f(p_out)*57.29578, q2f(q_out)*57.29578, q2f(r_out)*57.29578);

        // a few more steps so bias walk / MA buffer fill
        for (k = 0; k < 8; k = k + 1) begin
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            while (!done) @(posedge clk);
        end
        $display("s9 att_deg  = %10.4f %10.4f %10.4f",
                 q2f(roll_out)*57.29578, q2f(pitch_out)*57.29578, q2f(yaw_out)*57.29578);
        $display("s9 rate_dps = %10.4f %10.4f %10.4f",
                 q2f(p_out)*57.29578, q2f(q_out)*57.29578, q2f(r_out)*57.29578);

        $display("TB_SENSOR DONE");
        $finish;
    end
endmodule
