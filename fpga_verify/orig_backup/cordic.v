//----------------------------------------------------------------------
// cordic.v -- Iterative CORDIC: sin/cos (rotation) and atan2 (vectoring)
// Q16.16 fixed-point throughout
// 16 iterations, ~1 iteration per clock cycle
//
// sin/cos mode accepts ANY angle (full range): the input is first quadrant-
// folded into |z| <= pi/2 so the rotation-mode CORDIC (which only converges
// for |z| <= ~1.743 rad) is valid for all inputs.
//----------------------------------------------------------------------
module cordic (
    input  wire         clk,
    input  wire         rst,
    input  wire         start,          // pulse to start
    output reg          done,           // pulses when result ready

    // Operation select
    input  wire         op_sincos,      // 1 = sin/cos, 0 = atan2

    // Inputs for sin/cos mode: angle in radians Q16.16
    input  wire [31:0]  angle_in,

    // Outputs for sin/cos mode
    output wire [31:0]  sin_out,        // Q16.16
    output wire [31:0]  cos_out,        // Q16.16

    // Inputs for atan2 mode: y, x Q16.16
    input  wire [31:0]  atan2_y,
    input  wire [31:0]  atan2_x,

    // Output for atan2 mode: angle in radians Q16.16
    output wire [31:0]  atan2_out
);

    // CORDIC gain for 16 iterations: ~1.64676
    // Pre-scaled: multiply by 1/1.64676 = 0.607253
    localparam [31:0] CORDIC_SCALE = 32'sd39801;  // 0.607253 in Q16.16

    // pi/2 in Q16.16 = 1.5707963 -> 102944
    // pi  in Q16.16 = 3.14159265 -> 205887
    // Used to quadrant-fold the input angle of sin/cos into |z| <= pi/2 so the
    // rotation-mode CORDIC (converges for |z| <= ~1.743) is valid for all inputs.
    localparam signed [31:0] PI_OVER_2 = 32'sd102944;
    localparam signed [31:0] PI        = 32'sd205887;

    // atan(2^-i) lookup table, Q16.16
    // i=0: 45° = 0.785398 → 51472
    // i=1: 26.565° = 0.463648 → 30386
    // i=2: 14.036° = 0.244979 → 16053
    // i=3: 7.125° = 0.124355 → 8149
    // i=4: 3.576° = 0.062419 → 4090
    // i=5: 1.789° = 0.031234 → 2047
    // i=6: 0.895° = 0.015624 → 1024
    // i=7: 0.448° = 0.007812 → 512
    // i=8: 0.224° = 0.003906 → 256
    // i=9: 0.112° = 0.001953 → 128
    // i=10: 0.056° = 0.000977 → 64
    // i=11: 0.028° = 0.000488 → 32
    // i=12: 0.014° = 0.000244 → 16
    // i=13: 0.007° = 0.000122 → 8
    // i=14: 0.0035° = 0.000061 → 4
    // i=15: 0.0017° = 0.000031 → 2
    wire [31:0] atan_table [0:15];
    assign atan_table[ 0] = 32'd51472;
    assign atan_table[ 1] = 32'd30386;
    assign atan_table[ 2] = 32'd16054;
    assign atan_table[ 3] = 32'd8149;
    assign atan_table[ 4] = 32'd4091;
    assign atan_table[ 5] = 32'd2047;
    assign atan_table[ 6] = 32'd1024;
    assign atan_table[ 7] = 32'd512;
    assign atan_table[ 8] = 32'd256;
    assign atan_table[ 9] = 32'd128;
    assign atan_table[10] = 32'd64;
    assign atan_table[11] = 32'd32;
    assign atan_table[12] = 32'd16;
    assign atan_table[13] = 32'd8;
    assign atan_table[14] = 32'd4;
    assign atan_table[15] = 32'd2;

    // --- Registers ---
    reg [4:0]  iter;         // 0..16 (17 states: idle + 16 iterations)
    reg        running;
    reg        mode_sincos;  // latched mode
    reg        flip_xy;      // sin/cos: input folded past +/-pi/2 -> negate cos

    // X, Y, Z registers (wide to prevent overflow during computation)
    reg signed [33:0] x_reg, y_reg, z_reg;

    // Output registers
    reg [31:0] sin_reg, cos_reg, atan2_reg;

    wire signed [33:0] x_shifted = x_reg >>> iter;
    wire signed [33:0] y_shifted = y_reg >>> iter;

    always @(posedge clk) begin
        if (rst) begin
            iter       <= 5'd0;
            running    <= 1'b0;
            done       <= 1'b0;
            mode_sincos<= 1'b0;
            flip_xy    <= 1'b0;
            x_reg      <= 34'sd0;
            y_reg      <= 34'sd0;
            z_reg      <= 34'sd0;
            sin_reg    <= 32'd0;
            cos_reg    <= 32'd0;
            atan2_reg  <= 32'd0;
        end else begin
            done <= 1'b0;

            if (!running) begin
                if (start) begin
                    running     <= 1'b1;
                    mode_sincos <= op_sincos;
                    iter        <= 5'd0;
                    flip_xy     <= 1'b0;

                    if (op_sincos) begin
                        // Rotation mode: x=K, y=0, z=angle (folded).
                        // Quadrant-fold into |z| <= pi/2 with the rule:
                        //   angle in [ pi/2, 3pi/2]: z = pi - angle,   cos flips
                        //   angle in [-3pi/2,-pi/2]: z = -pi - angle,  cos flips
                        // For folded z: sin(angle) = sin(z),
                        //               cos(angle) = flip ? -cos(z) : cos(z).
                        // z is then in [-pi/2, pi/2], well within convergence.
                        if ($signed(angle_in) > PI_OVER_2) begin
                            flip_xy <= 1'b1;
                            z_reg   <= $signed(PI) - $signed(angle_in);
                        end else if ($signed(angle_in) < -PI_OVER_2) begin
                            flip_xy <= 1'b1;
                            z_reg   <= -$signed(PI) - $signed(angle_in);
                        end else begin
                            flip_xy <= 1'b0;
                            z_reg   <= $signed(angle_in);
                        end
                        x_reg <= $signed(CORDIC_SCALE);
                        y_reg <= 34'sd0;
                    end else begin
                        // Vectoring mode: x=atan2_x, y=atan2_y, z=0
                        x_reg <= $signed(atan2_x);
                        y_reg <= $signed(atan2_y);
                        z_reg <= 34'sd0;
                    end
                end
            end else begin
                if (iter == 5'd16) begin
                    running <= 1'b0;
                    done    <= 1'b1;
                    if (mode_sincos) begin
                        // Fold rule (verified at angle=120deg, etc.):
                        //   sin(angle) =  sin(z)   (unchanged)
                        //   cos(angle) = -cos(z)   (negated when folded)
                        sin_reg <= y_reg[31:0];
                        cos_reg <= flip_xy ? -x_reg[31:0] : x_reg[31:0];
                    end else begin
                        atan2_reg <= z_reg[31:0];
                    end
                end else begin
                    iter <= iter + 5'd1;

                    if (mode_sincos) begin
                        // Rotation mode
                        if (z_reg[33]) begin  // z < 0 → rotate clockwise
                            x_reg <= x_reg + y_shifted;
                            y_reg <= y_reg - x_shifted;
                            z_reg <= z_reg + {{2{atan_table[iter][31]}}, atan_table[iter]};
                        end else begin         // z >= 0 → rotate counter-clockwise
                            x_reg <= x_reg - y_shifted;
                            y_reg <= y_reg + x_shifted;
                            z_reg <= z_reg - {{2{atan_table[iter][31]}}, atan_table[iter]};
                        end
                    end else begin
                        // Vectoring mode
                        if (y_reg[33]) begin  // y < 0 → rotate clockwise
                            x_reg <= x_reg - y_shifted;
                            y_reg <= y_reg + x_shifted;
                            z_reg <= z_reg - {{2{atan_table[iter][31]}}, atan_table[iter]};
                        end else begin         // y >= 0 → rotate counter-clockwise
                            x_reg <= x_reg + y_shifted;
                            y_reg <= y_reg - x_shifted;
                            z_reg <= z_reg + {{2{atan_table[iter][31]}}, atan_table[iter]};
                        end
                    end
                end
            end
        end
    end

    assign sin_out   = sin_reg;
    assign cos_out   = cos_reg;
    assign atan2_out = atan2_reg;

endmodule
