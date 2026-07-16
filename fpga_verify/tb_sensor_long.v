`timescale 1ns / 1ps
//----------------------------------------------------------------------
// tb_sensor_long.v -- Long rest test for sensor_model with PASSTHROUGH=0
// Fails if any sample has |roll| or |pitch| > 15 deg (spikes, not noise).
//----------------------------------------------------------------------
module tb_sensor_long;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg start = 0;
    wire done;

    reg  signed [31:0] roll_in, pitch_in, yaw_in, p_in, q_in, r_in;
    wire signed [31:0] roll_out, pitch_out, yaw_out, p_out, q_out, r_out;

    sensor_model #(.PASSTHROUGH(0)) u_dut (
        .clk(clk), .rst(rst), .start(start), .done(done),
        .roll_in(roll_in), .pitch_in(pitch_in), .yaw_in(yaw_in),
        .p_in(p_in), .q_in(q_in), .r_in(r_in),
        .roll_out(roll_out), .pitch_out(pitch_out), .yaw_out(yaw_out),
        .p_out(p_out), .q_out(q_out), .r_out(r_out)
    );

    function real q2f;
        input signed [31:0] v;
        begin q2f = $itor(v) / 65536.0; end
    endfunction

    integer k, spikes, n;
    real ar, ap, ay, max_ar, max_ap, max_ay;
    // 15 deg in rad ≈ 0.2618 → Q16.16 ≈ 17157
    localparam signed [31:0] LIM = 32'sd17157;

    initial begin
        roll_in = 0; pitch_in = 0; yaw_in = 0;
        p_in = 0; q_in = 0; r_in = 0;
        spikes = 0; n = 0;
        max_ar = 0; max_ap = 0; max_ay = 0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        for (k = 0; k < 2000; k = k + 1) begin
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            while (!done) @(posedge clk);
            n = n + 1;
            ar = q2f(roll_out) * 57.29578;
            ap = q2f(pitch_out) * 57.29578;
            ay = q2f(yaw_out) * 57.29578;
            if (ar < 0) ar = -ar;
            if (ap < 0) ap = -ap;
            if (ay < 0) ay = -ay;
            if (ar > max_ar) max_ar = ar;
            if (ap > max_ap) max_ap = ap;
            if (ay > max_ay) max_ay = ay;
            if ((roll_out > LIM) || (roll_out < -LIM) ||
                (pitch_out > LIM) || (pitch_out < -LIM)) begin
                spikes = spikes + 1;
                if (spikes <= 10)
                    $display("SPIKE k=%0d roll=%.2f pitch=%.2f yaw=%.2f",
                             k, q2f(roll_out)*57.29578,
                             q2f(pitch_out)*57.29578,
                             q2f(yaw_out)*57.29578);
            end
        end

        $display("samples=%0d spikes(|rp|>15deg)=%0d", n, spikes);
        $display("max |roll|=%.3f |pitch|=%.3f |yaw|=%.3f deg",
                 max_ar, max_ap, max_ay);
        if (spikes != 0) begin
            $display("FAIL: spikes present");
            $finish(1);
        end
        if (max_ar > 10.0 || max_ap > 10.0) begin
            $display("FAIL: rest noise larger than expected");
            $finish(1);
        end
        $display("PASS: sensor noise at rest has no spike artifacts");
        $finish;
    end
endmodule
