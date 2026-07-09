//----------------------------------------------------------------------
// um7_tx.v -- CH Robotics UM7 "snp" packet emulator (FPGA side)
//
// Turns the Euler-angle + body-rate output of rov_model into a stream of
// valid UM7 binary packets on the UART, so that an STM32 running the
// existing um7.c parser sees a drop-in replacement for the physical IMU.
//
// What it transmits per start pulse (two packets, independent checksums):
//
//   Packet 1  -- Quaternion  (DREG_QUAT_AB=0x6D, DREG_QUAT_CD=0x6E)
//     's' 'n' 'p'  0xC8  0x6D
//       int16(w*29789.09)  int16(x*29789.09)        -- QUAT_AB (A=w, B=x)
//       int16(y*29789.09)  int16(z*29789.09)        -- QUAT_CD (C=y, D=z)
//     CK_HI CK_LO
//     PT = 0x80|0x40|(2<<2) = 0xC8  (has_data, is_batch, batch_len=2)
//     total = 7 + 8 = 15 bytes
//
//   Packet 2  -- Processed Gyro  (DREG_GYRO_PROC_X=0x61..0x63)
//     's' 'n' 'p'  0xCC  0x61
//       float32(p)  float32(q)  float32(r)          -- deg/s, big-endian
//       (rov_model outputs rad/s; converted to deg/s here via x180/pi)
//     CK_HI CK_LO
//     PT = 0x80|0x40|(3<<2) = 0xCC  (has_data, is_batch, batch_len=3)
//     total = 7 + 12 = 19 bytes
//
// Checksum (each packet): sum of 's','n','p',PT,ADDR and all payload bytes,
// masked to 16 bits, sent big-endian.  Matches um7.c::um7_try_parse().
//
// Data path:
//   Euler (phi=roll, theta=pitch, psi=yaw in radians, Q16.16)
//     -> 3 sincos(half-angle) via a dedicated cordic instance
//     -> 2-stage Euler->quat combine (ZYX convention, matches um7.c)
//     -> int16 scale (x29789.09) for quat
//   Body rates p,q,r (rad/s, Q16.16)
//     -> rad/s to deg/s (x 180/pi = 57.29578, FP_180_OVER_PI)
//     -> Q16->float32 for gyro
//     -> frame + checksum + push byte-by-byte through uart_tx_byte_if
//
// start pulse latches the 6 inputs; done pulses when both packets are gone.
// One start per rov_model step (~50 Hz).  At 115200 baud a 34-byte burst
// takes ~2.9 ms, well inside the 20 ms step period.
//----------------------------------------------------------------------
`include "rov_params.vh"

module um7_tx (
    input  wire        clk,
    input  wire        rst,

    // Trigger: pulse to latch inputs and begin a transmission cycle.
    input  wire        start,
    output reg         done,

    // ROV state inputs (Q16.16).  roll/pitch/yaw in RADIANS (not degrees),
    // p/q/r body rates in rad/s (converted to deg/s before TX).
    input  wire signed [31:0] roll,   // phi
    input  wire signed [31:0] pitch,  // theta
    input  wire signed [31:0] yaw,    // psi
    input  wire signed [31:0] p,
    input  wire signed [31:0] q,
    input  wire signed [31:0] r,

    // Byte-level UART interface (same handshake as uart_print_hex)
    output reg         send_byte,
    output reg  [7:0]  byte_out,
    input  wire        byte_busy,
    input  wire        byte_done
);

    //==================================================================
    // Fixed-point helpers (same convention as rov_model.v: Q16.16)
    //==================================================================
    // Q16.16 multiply (truncate). Pure fmul states are single-depth;
    // multi-fmul+add paths are split across FSM cycles (ST_Q2A/B, etc.).
    function [31:0] fmul;
        input signed [31:0] a, b;
        reg signed [63:0] prod;
        begin
            prod = a * b;
            fmul = prod[47:16];
        end
    endfunction

    // 0.5 in Q16.16
    localparam signed [31:0] FP_HALF = 32'sd32768;

    // Quaternion int16 scale: 29789.09091 in Q16.16 = 29789.09091 * 65536
    //   = 1952257861.0 (rounds to 1952257861)
    localparam signed [31:0] FP_QUAT_SCALE = 32'sd1952257861; // 29789.09091

    localparam signed [31:0] INT16_MAX =  32'sd32767;
    localparam signed [31:0] INT16_MIN = -32'sd32768;

    //==================================================================
    // Dedicated CORDIC (sin/cos only). Independent of rov_model's CORDIC
    // so the two never contend; cost is ~50 LUTs, no DSP.
    //==================================================================
    reg         cord_start;
    reg         cord_sincos;
    reg  [31:0] cord_angle;
    wire        cord_done;
    wire [31:0] cord_sin, cord_cos;

    cordic u_cord (
        .clk(clk), .rst(rst),
        .start(cord_start), .done(cord_done),
        .op_sincos(cord_sincos),
        .angle_in(cord_angle),
        .sin_out(cord_sin), .cos_out(cord_cos),
        .atan2_y(32'd0), .atan2_x(32'd0),
        .atan2_out()
    );

    //==================================================================
    // Latched inputs + half-angle trig registers (6 distinct values)
    //==================================================================
    reg signed [31:0] l_phi, l_th, l_ps;     // radians
    reg signed [31:0] l_p,  l_q,  l_r;       // rad/s

    // half-angle cos/sin. cr,sr = roll(phi)/2 ; cth,sth = pitch(theta)/2 ;
    // cy,sy = yaw(psi)/2.  Names chosen to avoid clash with FSM reg `state`.
    reg signed [31:0] cr, sr, cth, sth, cy, sy;

    // stage-1 products of the Euler->quat combine
    reg signed [31:0] m_crct, m_srst, m_srct, m_crst;

    // assembled quaternion components (Q16.16)
    reg signed [31:0] qw, qx, qy, qz;

    // packed payload bytes for both packets
    reg [7:0] q_payload [0:7];   // 4 x int16 BE
    reg [7:0] g_payload [0:11];  // 3 x float32 BE

    // Intermediate product / float registers for multi-cycle packing.
    // Timing (impl @ 100 MHz): a single-cycle fmul(DSP cascade) →
    // q16_to_float (priority-encoder + shift) → g_payload was WNS=-4.86 ns.
    // Each arithmetic step is now its own FSM cycle.
    reg signed [31:0] q2_ww, q2_wx, q2_wy, q2_wz;   // ST_Q2a products
    reg signed [31:0] q2_xw, q2_xx, q2_yw, q2_yx;
    reg signed [31:0] qs_w, qs_x, qs_y, qs_z;       // quat * scale
    reg signed [31:0] deg_p, deg_q, deg_r;          // rad/s * (180/pi)
    reg [31:0]        fp;                            // q16_to_float result
    reg [15:0]        iw, ix, iy, iz;                // sat_int16 temps

    //==================================================================
    // FSM
    //==================================================================
    // State encoding widened to 5 bits with spare codes for the split
    // pack / quat stages (was one-cycle fmul+float / fmul+add).
    localparam [4:0]
        ST_IDLE      = 5'd0,
        ST_PHI       = 5'd1,   ST_W_PHI   = 5'd2,
        ST_THETA     = 5'd3,   ST_W_THETA = 5'd4,
        ST_PSI       = 5'd5,   ST_W_PSI   = 5'd6,
        ST_Q1        = 5'd7,   // stage-1 products (4 fmul, registered)
        ST_Q2A       = 5'd8,   // stage-2 products (8 fmul, registered)
        ST_Q2B       = 5'd16,  // stage-2 add/sub -> qw..qz
        ST_PACK_Q_MUL= 5'd9,   // quat * FP_QUAT_SCALE
        ST_PACK_Q_SAT= 5'd17,  // saturate + pack int16 BE
        ST_PACK_G_MUL= 5'd10,  // p,q,r * (180/pi)
        ST_PACK_G_P  = 5'd18,  // float(deg_p) -> g_payload[0:3]
        ST_PACK_G_Q  = 5'd19,  // float(deg_q) -> g_payload[4:7]
        ST_PACK_G_R  = 5'd20,  // float(deg_r) -> g_payload[8:11]
        ST_TX_Q      = 5'd11,
        ST_TX_Q_WAIT = 5'd12,
        ST_TX_G      = 5'd13,
        ST_TX_G_WAIT = 5'd14,
        ST_DONE      = 5'd15;

    reg [4:0] state;
    reg [4:0] tx_idx;          // byte index within current packet
    reg        tx_pkt;         // 0 = quaternion packet, 1 = gyro packet
    reg  [7:0] tx_byte;        // combinational byte selected by {tx_pkt, tx_idx}
    // 2-cycle hold on pure-fmul states (pairs with multicycle in constraint.xdc)
    reg        mul_hold;

    //==================================================================
    // Q16.16 -> IEEE-754 float32
    //   value = v / 65536.0 ; maps |v| < 2^-14 (v==0) to signed zero.
    //==================================================================
    function [31:0] q16_to_float;
        input signed [31:0] v;
        reg        sign;
        reg [31:0] abs_v;
        reg [31:0] shifted;
        integer    lz;         // leading-zero count (0..31)
        reg [7:0]  exp8;
        reg [22:0] mant;
        begin
            sign  = v[31];
            abs_v = sign ? (~v + 1'b1) : v;
            if (abs_v == 32'd0) begin
                q16_to_float = {sign, 8'd0, 23'd0};          // +-0
            end else begin
                // 32-bit priority encoder -> lz (leading zeros of abs_v)
                lz = 0;
                shifted = abs_v;
                if (shifted[31:16] == 16'd0) begin lz = lz + 16; shifted = {shifted[15:0], 16'd0}; end
                if (shifted[31:24] ==  8'd0) begin lz = lz +  8; shifted = {shifted[23:0],  8'd0}; end
                if (shifted[31:28] ==  4'd0) begin lz = lz +  4; shifted = {shifted[27:0],  4'd0}; end
                if (shifted[31:30] ==  2'd0) begin lz = lz +  2; shifted = {shifted[29:0],  2'd0}; end
                if (shifted[31]    ==  1'd0) begin lz = lz +  1; shifted = {shifted[30:0],  1'd0}; end
                // Leading '1' is now at bit 31. In abs_v it was at bit (31-lz);
                // the Q16.16 binary point sits 16 below bit0, so its place
                // value was 2^((31-lz) - 16) = 2^(15 - lz).
                // IEEE biased exponent = (15 - lz) + 127 = 142 - lz.
                exp8 = 8'd142 - lz[7:0];
                mant = shifted[30:8];   // 23 bits below the leading '1'
                q16_to_float = {sign, exp8, mant};
            end
        end
    endfunction

    //==================================================================
    // Saturate a pre-scaled Q16.16 value to int16 (no multiply here).
    //==================================================================
    function [15:0] sat_int16;
        input signed [31:0] scaled;
        begin
            if (scaled > INT16_MAX)       sat_int16 = 16'h7FFF;
            else if (scaled < INT16_MIN)  sat_int16 = 16'h8000;
            else                          sat_int16 = scaled[15:0];
        end
    endfunction

    //==================================================================
    // Main FSM
    //==================================================================
    always @(posedge clk) begin
        if (rst) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            cord_start <= 1'b0;
            cord_sincos<= 1'b0;
            send_byte  <= 1'b0;
            byte_out   <= 8'd0;
            tx_idx     <= 5'd0;
            tx_pkt     <= 1'b0;
            mul_hold   <= 1'b0;
        end else begin
            done       <= 1'b0;
            cord_start <= 1'b0;
            send_byte  <= 1'b0;

            case (state)
                //--------------------------------------------------------
                ST_IDLE: begin
                    if (start) begin
                        l_phi <= roll;
                        l_th  <= pitch;
                        l_ps  <= yaw;
                        l_p   <= p;
                        l_q   <= q;
                        l_r   <= r;
                        state <= ST_PHI;
                    end
                end

                //--------------------------------------------------------
                // sin/cos(phi/2)
                ST_PHI: begin
                    cord_start  <= 1'b1;
                    cord_sincos <= 1'b1;
                    cord_angle  <= fmul(l_phi, FP_HALF);
                    state       <= ST_W_PHI;
                end
                ST_W_PHI: if (cord_done) begin
                    cr <= cord_cos;
                    sr <= cord_sin;
                    state <= ST_THETA;
                end

                //--------------------------------------------------------
                // sin/cos(theta/2)
                ST_THETA: begin
                    cord_start  <= 1'b1;
                    cord_sincos <= 1'b1;
                    cord_angle  <= fmul(l_th, FP_HALF);
                    state       <= ST_W_THETA;
                end
                ST_W_THETA: if (cord_done) begin
                    cth <= cord_cos;
                    sth <= cord_sin;
                    state <= ST_PSI;
                end

                //--------------------------------------------------------
                // sin/cos(psi/2)
                ST_PSI: begin
                    cord_start  <= 1'b1;
                    cord_sincos <= 1'b1;
                    cord_angle  <= fmul(l_ps, FP_HALF);
                    state       <= ST_W_PSI;
                end
                ST_W_PSI: if (cord_done) begin
                    cy <= cord_cos;
                    sy <= cord_sin;
                    state <= ST_Q1;
                end

                //--------------------------------------------------------
                // Euler->quat stage 1: the four roll*theta products
                //   w,z need (cr*cth) and (sr*sth)
                //   x,y need (sr*cth) and (cr*sth)
                ST_Q1: begin
                    m_crct <= fmul(cr,  cth);
                    m_srst <= fmul(sr,  sth);
                    m_srct <= fmul(sr,  cth);
                    m_crst <= fmul(cr,  sth);
                    if (mul_hold) begin mul_hold <= 1'b0; state <= ST_Q2A; end
                    else          mul_hold <= 1'b1;
                end

                //--------------------------------------------------------
                // Euler->quat stage 2a: register all 8 products
                // (was 2*fmul + add in one cycle -- fails at 100 MHz)
                ST_Q2A: begin
                    q2_ww <= fmul(m_crct, cy);
                    q2_wx <= fmul(m_srst, sy);
                    q2_xw <= fmul(m_srct, cy);
                    q2_xx <= fmul(m_crst, sy);
                    q2_yw <= fmul(m_crst, cy);
                    q2_yx <= fmul(m_srct, sy);
                    q2_wz <= fmul(m_crct, sy);
                    q2_wy <= fmul(m_srst, cy);
                    if (mul_hold) begin mul_hold <= 1'b0; state <= ST_Q2B; end
                    else          mul_hold <= 1'b1;
                end

                //--------------------------------------------------------
                // Euler->quat stage 2b (ZYX convention, matches um7.c):
                //   w = m_crct*cy + m_srst*sy
                //   x = m_srct*cy - m_crst*sy
                //   y = m_crst*cy + m_srct*sy
                //   z = m_crct*sy - m_srst*cy
                ST_Q2B: begin
                    qw <= q2_ww + q2_wx;
                    qx <= q2_xw - q2_xx;
                    qy <= q2_yw + q2_yx;
                    qz <= q2_wz - q2_wy;
                    state <= ST_PACK_Q_MUL;
                end

                //--------------------------------------------------------
                // Pack quaternion: cycle 1 = scale (fmul only)
                ST_PACK_Q_MUL: begin
                    qs_w <= fmul(qw, FP_QUAT_SCALE);
                    qs_x <= fmul(qx, FP_QUAT_SCALE);
                    qs_y <= fmul(qy, FP_QUAT_SCALE);
                    qs_z <= fmul(qz, FP_QUAT_SCALE);
                    if (mul_hold) begin mul_hold <= 1'b0; state <= ST_PACK_Q_SAT; end
                    else          mul_hold <= 1'b1;
                end

                //--------------------------------------------------------
                // Pack quaternion: cycle 2 = saturate + pack int16 BE
                ST_PACK_Q_SAT: begin
                    // Blocking temps: Verilog cannot bit-select a function result.
                    iw = sat_int16(qs_w);
                    ix = sat_int16(qs_x);
                    iy = sat_int16(qs_y);
                    iz = sat_int16(qs_z);
                    q_payload[0] <= iw[15:8];
                    q_payload[1] <= iw[7:0];
                    q_payload[2] <= ix[15:8];
                    q_payload[3] <= ix[7:0];
                    q_payload[4] <= iy[15:8];
                    q_payload[5] <= iy[7:0];
                    q_payload[6] <= iz[15:8];
                    q_payload[7] <= iz[7:0];
                    state <= ST_PACK_G_MUL;
                end

                //--------------------------------------------------------
                // Pack gyro: cycle 1 = rad/s -> deg/s (fmul only)
                ST_PACK_G_MUL: begin
                    deg_p <= fmul(l_p, `FP_180_OVER_PI);
                    deg_q <= fmul(l_q, `FP_180_OVER_PI);
                    deg_r <= fmul(l_r, `FP_180_OVER_PI);
                    if (mul_hold) begin mul_hold <= 1'b0; state <= ST_PACK_G_P; end
                    else          mul_hold <= 1'b1;
                end

                //--------------------------------------------------------
                // Pack gyro: one axis per cycle so q16_to_float is alone
                // on the path into g_payload (no DSP cascade ahead of it).
                ST_PACK_G_P: begin
                    fp = q16_to_float(deg_p);
                    g_payload[0] <= fp[31:24];
                    g_payload[1] <= fp[23:16];
                    g_payload[2] <= fp[15:8];
                    g_payload[3] <= fp[7:0];
                    state <= ST_PACK_G_Q;
                end
                ST_PACK_G_Q: begin
                    fp = q16_to_float(deg_q);
                    g_payload[4] <= fp[31:24];
                    g_payload[5] <= fp[23:16];
                    g_payload[6] <= fp[15:8];
                    g_payload[7] <= fp[7:0];
                    state <= ST_PACK_G_R;
                end
                ST_PACK_G_R: begin
                    fp = q16_to_float(deg_r);
                    g_payload[8]  <= fp[31:24];
                    g_payload[9]  <= fp[23:16];
                    g_payload[10] <= fp[15:8];
                    g_payload[11] <= fp[7:0];
                    tx_idx <= 5'd0;
                    state  <= ST_TX_Q;
                end

                //--------------------------------------------------------
                // Transmit quaternion packet (15 bytes).
                //   idx 0..4: 's' 'n' 'p' 0xC8 0x6D
                //   idx 5..12: q_payload[0..7]
                //   idx 13,14: checksum HI, LO
                // Handshake (mirrors uart_print_hex): when the byte channel
                // is free, present a byte + pulse send_byte for one cycle;
                // then wait for byte_done before advancing the index.
                // The actual byte is selected combinationally by tx_byte
                // (see the always @(*) block below) using {tx_pkt, tx_idx}.
                ST_TX_Q: begin
                    tx_pkt <= 1'b0;                 // 0 = quaternion packet
                    if (!byte_busy && !send_byte) begin
                        byte_out  <= tx_byte;
                        send_byte <= 1'b1;           // 1-cycle pulse
                        state     <= ST_TX_Q_WAIT;
                    end
                end
                ST_TX_Q_WAIT: begin
                    if (byte_done) begin
                        if (tx_idx == 5'd14) begin
                            tx_idx <= 5'd0;
                            state  <= ST_TX_G;
                        end else begin
                            tx_idx <= tx_idx + 5'd1;
                            state  <= ST_TX_Q;
                        end
                    end
                end

                //--------------------------------------------------------
                // Transmit gyro packet (19 bytes).
                //   idx 0..4: 's' 'n' 'p' 0xCC 0x61
                //   idx 5..16: g_payload[0..11]
                //   idx 17,18: checksum HI, LO
                ST_TX_G: begin
                    tx_pkt <= 1'b1;                 // 1 = gyro packet
                    if (!byte_busy && !send_byte) begin
                        byte_out  <= tx_byte;
                        send_byte <= 1'b1;           // 1-cycle pulse
                        state     <= ST_TX_G_WAIT;
                    end
                end
                ST_TX_G_WAIT: begin
                    if (byte_done) begin
                        if (tx_idx == 5'd18) begin
                            tx_idx <= 5'd0;
                            state  <= ST_DONE;
                        end else begin
                            tx_idx <= tx_idx + 5'd1;
                            state  <= ST_TX_G;
                        end
                    end
                end

                //--------------------------------------------------------
                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    //==================================================================
    // Packet constants + checksum + byte lookup (combinational)
    //==================================================================
    localparam [7:0] UM7_S   = 8'h73;   // 's'
    localparam [7:0] UM7_N   = 8'h6E;   // 'n'
    localparam [7:0] UM7_P   = 8'h70;   // 'p'
    localparam [7:0] PT_QUAT = 8'hC8;   // has_data|is_batch|(batch_len=2)<<2
    localparam [7:0] AD_QUAT = 8'h6D;
    localparam [7:0] PT_GYRO = 8'hCC;   // has_data|is_batch|(batch_len=3)<<2
    localparam [7:0] AD_GYRO = 8'h61;

    //==================================================================
    // Combinational byte selector + checksums.
    // Implemented as an always @(*) block (NOT functions) because Verilog
    // functions may not reference module-scope memory arrays (q_payload /
    // g_payload). Procedural blocks in always @(*) can.
    //   tx_pkt=0 -> quaternion packet (15 bytes, idx 0..14)
    //   tx_pkt=1 -> gyro packet       (19 bytes, idx 0..18)
    //==================================================================
    // 16-bit checksum accumulators (combinational)
    wire [16:0] csum_q_sum = {1'b0, UM7_S} + UM7_N + UM7_P + PT_QUAT + AD_QUAT
                           + q_payload[0] + q_payload[1] + q_payload[2] + q_payload[3]
                           + q_payload[4] + q_payload[5] + q_payload[6] + q_payload[7];
    wire [15:0] csum_q_val = csum_q_sum[15:0];

    wire [16:0] csum_g_sum = {1'b0, UM7_S} + UM7_N + UM7_P + PT_GYRO + AD_GYRO
                           + g_payload[0]  + g_payload[1]  + g_payload[2]  + g_payload[3]
                           + g_payload[4]  + g_payload[5]  + g_payload[6]  + g_payload[7]
                           + g_payload[8]  + g_payload[9]  + g_payload[10] + g_payload[11];
    wire [15:0] csum_g_val = csum_g_sum[15:0];

    always @(*) begin
        tx_byte = 8'h00;
        if (tx_pkt == 1'b0) begin
            // ---- quaternion packet (idx 0..14) ----
            case (tx_idx)
                5'd0:  tx_byte = UM7_S;
                5'd1:  tx_byte = UM7_N;
                5'd2:  tx_byte = UM7_P;
                5'd3:  tx_byte = PT_QUAT;
                5'd4:  tx_byte = AD_QUAT;
                5'd5:  tx_byte = q_payload[0];
                5'd6:  tx_byte = q_payload[1];
                5'd7:  tx_byte = q_payload[2];
                5'd8:  tx_byte = q_payload[3];
                5'd9:  tx_byte = q_payload[4];
                5'd10: tx_byte = q_payload[5];
                5'd11: tx_byte = q_payload[6];
                5'd12: tx_byte = q_payload[7];
                5'd13: tx_byte = csum_q_val[15:8];
                5'd14: tx_byte = csum_q_val[7:0];
                default: tx_byte = 8'h00;
            endcase
        end else begin
            // ---- gyro packet (idx 0..18) ----
            case (tx_idx)
                5'd0:  tx_byte = UM7_S;
                5'd1:  tx_byte = UM7_N;
                5'd2:  tx_byte = UM7_P;
                5'd3:  tx_byte = PT_GYRO;
                5'd4:  tx_byte = AD_GYRO;
                5'd5:  tx_byte = g_payload[0];
                5'd6:  tx_byte = g_payload[1];
                5'd7:  tx_byte = g_payload[2];
                5'd8:  tx_byte = g_payload[3];
                5'd9:  tx_byte = g_payload[4];
                5'd10: tx_byte = g_payload[5];
                5'd11: tx_byte = g_payload[6];
                5'd12: tx_byte = g_payload[7];
                5'd13: tx_byte = g_payload[8];
                5'd14: tx_byte = g_payload[9];
                5'd15: tx_byte = g_payload[10];
                5'd16: tx_byte = g_payload[11];
                5'd17: tx_byte = csum_g_val[15:8];
                5'd18: tx_byte = csum_g_val[7:0];
                default: tx_byte = 8'h00;
            endcase
        end
    end

endmodule
