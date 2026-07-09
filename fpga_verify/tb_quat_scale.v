`timescale 1ns / 1ps
// Quick check: identity quaternion (w=1,x=y=z=0) must pack to
// int16 ≈ (29789, 0, 0, 0), NOT (±32767,...).
module tb_quat_scale;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg start = 0;
    wire done;
    wire send_byte;
    wire [7:0] byte_out;

    wire busy;
    um7_tx u_dut (
        .clk(clk), .rst(rst),
        .start(start), .done(done), .busy(busy),
        .roll(32'sd0), .pitch(32'sd0), .yaw(32'sd0),
        .p(32'sd0), .q(32'sd0), .r(32'sd0),
        .send_byte(send_byte), .byte_out(byte_out),
        .byte_busy(1'b0), .byte_done(send_byte)  // instant ack each byte
    );

    integer i;
    reg [7:0] bytes [0:33];
    integer nbyte;
    reg [15:0] w16, x16, y16, z16;
    initial begin
        nbyte = 0;
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // collect all TX bytes
        while (!done) begin
            @(posedge clk);
            if (send_byte) begin
                bytes[nbyte] = byte_out;
                nbyte = nbyte + 1;
            end
        end
        $display("collected %0d bytes", nbyte);
        // quat packet: idx 0..4 header, 5..12 payload (w,x,y,z BE int16)
        w16 = {bytes[5], bytes[6]};
        x16 = {bytes[7], bytes[8]};
        y16 = {bytes[9], bytes[10]};
        z16 = {bytes[11], bytes[12]};
        $display("quat int16 w=%0d x=%0d y=%0d z=%0d",
                 $signed(w16), $signed(x16), $signed(y16), $signed(z16));
        $display("expect w~29789, x=y=z~0 (identity at RPY=0)");
        // CORDIC residual of a few LSBs on x,y,z is normal; w must be ~29789.
        if ($signed(w16) > 29000 && $signed(w16) < 30500 &&
            $signed(x16) > -50 && $signed(x16) < 50 &&
            $signed(y16) > -50 && $signed(y16) < 50 &&
            $signed(z16) > -50 && $signed(z16) < 50)
            $display("PASS: identity quaternion packing (w~29789, xyz~0)");
        else if ($signed(w16) == 32767 || $signed(w16) == -32768)
            $display("FAIL: still saturating to int16 max (old bug)");
        else
            $display("CHECK: unexpected values");
        $finish;
    end
endmodule
