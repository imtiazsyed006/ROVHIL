//----------------------------------------------------------------------
// duty_to_force.v -- Convert 8 CCR-scaled duty values -> 8 normalized forces
//
// Implements the exact Python/MATLAB per-channel algorithm:
//   1. duty counts (= pwm_ctrl 4400..7600, 4 counts per us) clamped
//   2. Forward  (pwm > 1501 us): u_sat = +force / max_fwd_force
//      Reverse  (pwm < 1499 us): u_sat = -force_mag / max_rev_force
//      Deadzone (pwm ~ 1500 us): u_sat = 0
//   3. Clamp u_sat magnitude to [0, 65536]  (= 1.0 in Q16.16)
//   4. Per-channel negation undo: if chi != 5, u_sat = -u_sat
//      (STM32 negates every channel except ch6)
//
// LUT interpolation runs on the raw duty-count grid (quarter-us steps,
// 16 counts per 4-us LUT entry), matching np.interp to full precision:
//   force = f0 + frac16 * (f1 - f0) / 16
//
// Uses dual thrust-curve LUTs identical to the Python (xlsx) LUT:
//   fwd_lut.mem -- 92 entries, PWM 1536:4:1900, force kgf Q16.16
//   rev_lut.mem -- 92 entries, PWM 1100:4:1464, force magnitude kgf Q16.16
//
// Output: 8 x 32-bit signed Q16.16 normalized forces, packed [255:0].
//----------------------------------------------------------------------
`include "rov_params.vh"

module duty_to_force (
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    output reg          done,

    input  wire [127:0] duty_flat,
    input  wire [7:0]   ch_active,

    output reg  [255:0] u_flat
);

    //==================================================================
    // Dual thrust-curve block RAMs
    //==================================================================
    reg signed [31:0] fwd_mem [0:`FWD_LUT_ENTRIES-1];
    reg signed [31:0] rev_mem [0:`REV_LUT_ENTRIES-1];
    initial begin
        $readmemh("fwd_lut.mem", fwd_mem);
        $readmemh("rev_lut.mem", rev_mem);
    end

    //==================================================================
    // Shared divider -- 48-cycle shift-subtract
    //   quotient = (dividend_q16 << 16) / divisor_q16
    //==================================================================
    reg [31:0] dvdividend;    // force for divider
    reg [31:0] dvdivisor;     // max_force for divider
    reg        dvs;           // 1-cycle pulse to launch
    reg        dvb;           // divider busy
    reg [5:0]  dvc;           // iteration count 0..48
    reg [47:0] dv_div;        // dividend shift register
    reg [47:0] dv_rem;        // running remainder
    reg [47:0] dvqr;          // running quotient
    reg [31:0] dvden;         // divisor (max_force)
    wire       dv_done = dvb && (dvc == 6'd49);
    wire       dv_bit  = dv_div[47];

    always @(posedge clk) begin
        if (rst) begin
            dvb <= 1'b0; dvc <= 6'd0; dv_div <= 48'd0;
            dv_rem <= 48'd0; dvqr <= 48'd0; dvden <= 32'd0;
        end else begin
            if (dvs && !dvb) begin
                dvb    <= 1'b1;
                dvc    <= 6'd0;
                dv_div <= {dvdividend, 16'd0};
                dv_rem <= 48'd0;
                dvqr   <= 48'd0;
                dvden  <= (dvdivisor == 32'd0) ? 32'd1 : dvdivisor;
            end else if (dvb) begin
                if (dvc < 6'd48) begin
                    dvc    <= dvc + 6'd1;
                    dv_div <= {dv_div[46:0], 1'b0};
                    if ({dv_rem[46:0], dv_bit} >= {16'd0, dvden}) begin
                        dv_rem <= {dv_rem[46:0], dv_bit} - {16'd0, dvden};
                        dvqr   <= {dvqr[46:0], 1'b1};
                    end else begin
                        dv_rem <= {dv_rem[46:0], dv_bit};
                        dvqr   <= {dvqr[46:0], 1'b0};
                    end
                end else if (dvc == 6'd48) begin
                    // round-to-nearest: bump quotient if 2*rem >= divisor
                    if ({dv_rem[46:0], 1'b0} >= {16'd0, dvden})
                        dvqr <= dvqr + 48'd1;
                    dvc <= 6'd49;
                end else begin
                    dvb <= 1'b0;
                end
            end
        end
    end

    //==================================================================
    // FSM states
    //==================================================================
    // 4-bit state: ST_INTERP split into mul + add for 100 MHz timing
    // (f0+(frac*(f1-f0))>>4 was WNS≈-0.8 ns as a single cycle).
    localparam [3:0]
        ST_IDLE    = 4'd0,
        ST_DIR     = 4'd1,   // direction, LUT idx, frac from duty counts
        ST_READ0   = 4'd2,   // read LUT[idx]
        ST_READ1   = 4'd3,   // read LUT[idx+1]
        ST_INTERP0 = 4'd4,   // delta = f1-f0; prod = frac*delta
        ST_INTERP1 = 4'd8,   // force = f0 + (prod>>>4)
        ST_DIV     = 4'd5,   // launch divider
        ST_WDIV    = 4'd6,   // wait for divider
        ST_SIGN    = 4'd7;   // apply sign + per-channel negation, then next/done

    reg [3:0]  st;
    reg [2:0]  chi;
    reg signed [31:0] interp_prod;  // frac * (f1-f0), registered

    // Per-channel registers
    reg        fwd_r;         // 1=forward, 0=reverse
    reg        dead_r;        // 1=deadzone or inactive
    reg        rev_r;         // 1=reverse -> negative u_sat
    reg [31:0] maxf_r;        // FP_MAX_FWD_FORCE or FP_MAX_REV_FORCE
    reg [7:0]  idx_r;         // LUT base index
    reg [7:0]  idx1_r;        // idx+1, clamped to last entry
    reg [3:0]  frac_r;        // interpolation fraction (0..15 = quarter-us)
    reg signed [31:0] f0_r, f1_r;  // LUT read values

    //==================================================================
    // Combinational: LUT index math on the duty-count grid
    //   duty counts: 4 per us -> 1100us = 4400, 1500us = 6000, 1900us = 7600
    //   LUT step = 4us = 16 counts; frac = counts within a LUT step (0..15)
    //==================================================================
    wire [15:0] duty_w = duty_flat[chi*16 +: 16];

    wire [15:0] duty_clamp =
        (duty_w < 16'd4400) ? 16'd4400 :
        (duty_w > 16'd7600) ? 16'd7600 :
                              duty_w;

    // Forward offset: counts past 1536us (=6144), clamped to 0..1456
    wire [15:0] fwd_off = (duty_clamp < 16'd6144) ? 16'd0 :
                          (duty_clamp - 16'd6144);
    // Reverse offset: counts past 1100us (=4400), clamped to 0..1456 (1464us)
    wire [15:0] rev_off = (duty_clamp > 16'd5856) ? 16'd1456 :
                          (duty_clamp - 16'd4400);

    // Forward indices (0..91, idx+1 capped at the last entry 91)
    wire [7:0] fwd_idx_w  = fwd_off[11:4];
    wire [7:0] fwd_idx1_w = (fwd_off[11:4] >= 8'd91) ? 8'd91 : (fwd_off[11:4] + 8'd1);
    wire [3:0] fwd_frac_w = fwd_off[3:0];

    // Reverse indices (0..91, idx+1 capped at the last entry 91)
    wire [7:0] rev_idx_w  = rev_off[11:4];
    wire [7:0] rev_idx1_w = (rev_off[11:4] >= 8'd91) ? 8'd91 : (rev_off[11:4] + 8'd1);
    wire [3:0] rev_frac_w = rev_off[3:0];

    //==================================================================
    // Main FSM
    //==================================================================
    always @(posedge clk) begin
        if (rst) begin
            st     <= ST_IDLE;
            done   <= 1'b0;
            dvs    <= 1'b0;
            chi    <= 3'd0;
            u_flat <= 256'd0;
        end else begin
            done <= 1'b0;
            dvs  <= 1'b0;

            case (st)

                ST_IDLE: begin
                    if (start) begin
                        chi <= 3'd0;
                        st  <= ST_DIR;
                    end
                end

                //----------------------------------------------------------
                // ST_DIR: determine direction & compute LUT indices
                //----------------------------------------------------------
                ST_DIR: begin
                    // Dead band: +/-1 us (4 counts) around 1500us. The STM32
                    // only ever emits 1500us (u=0) inside 1465..1535us, so
                    // this exactly reproduces Python's zero output there.
                    if (!ch_active[chi] ||
                        (duty_clamp >= 16'd5996 && duty_clamp <= 16'd6004)) begin
                        // Dead zone or inactive -> u_sat = 0
                        dead_r <= 1'b1;
                        fwd_r  <= 1'b0;
                        rev_r  <= 1'b0;
                    end else if (duty_clamp > 16'd6004) begin
                        // ---- FORWARD ----
                        dead_r   <= 1'b0;
                        fwd_r    <= 1'b1;
                        rev_r    <= 1'b0;
                        maxf_r   <= `FP_MAX_FWD_FORCE;
                        idx_r    <= fwd_idx_w;
                        idx1_r   <= fwd_idx1_w;
                        frac_r   <= fwd_frac_w;
                    end else begin
                        // ---- REVERSE ----
                        dead_r   <= 1'b0;
                        fwd_r    <= 1'b0;
                        rev_r    <= 1'b1;
                        maxf_r   <= `FP_MAX_REV_FORCE;
                        idx_r    <= rev_idx_w;
                        idx1_r   <= rev_idx1_w;
                        frac_r   <= rev_frac_w;
                    end
                    st <= ST_READ0;
                end

                //----------------------------------------------------------
                // ST_READ0: read LUT[idx] (registered address)
                //----------------------------------------------------------
                ST_READ0: begin
                    f0_r <= fwd_r ? fwd_mem[idx_r] : rev_mem[idx_r];
                    st   <= ST_READ1;
                end
                // ST_READ1: read LUT[idx+1]
                ST_READ1: begin
                    f1_r <= fwd_r ? fwd_mem[idx1_r] : rev_mem[idx1_r];
                    st   <= ST_INTERP0;
                end

                //----------------------------------------------------------
                // ST_INTERP0: register frac*(f1-f0)  (one multiply depth)
                // ST_INTERP1: force = f0 + prod/16  (add only)
                //----------------------------------------------------------
                ST_INTERP0: begin
                    if (dead_r)
                        interp_prod <= 32'sd0;
                    else
                        interp_prod <= $signed({1'b0, frac_r}) * (f1_r - f0_r);
                    st <= ST_INTERP1;
                end
                ST_INTERP1: begin
                    if (dead_r)
                        dvdividend <= 32'd0;
                    else
                        dvdividend <= f0_r + (interp_prod >>> 4);
                    dvdivisor <= maxf_r;
                    st <= ST_DIV;
                end

                //----------------------------------------------------------
                // ST_DIV: launch divider or skip if dead/inactive
                //----------------------------------------------------------
                ST_DIV: begin
                    if (dead_r || !ch_active[chi]) begin
                        u_flat[chi*32 +: 32] <= 32'sd0;
                        st <= ST_SIGN;  // ST_SIGN handles next/done
                    end else if (dvdividend == 32'd0) begin
                        // force=0 -> u_sat=0
                        u_flat[chi*32 +: 32] <= 32'sd0;
                        st <= ST_SIGN;
                    end else begin
                        dvs <= 1'b1;
                        st  <= ST_WDIV;
                    end
                end

                //----------------------------------------------------------
                // ST_WDIV: wait for divider, then apply sign
                //   Quotient can legitimately reach 65536 (= 1.0 exactly at
                //   full-scale PWM), so the magnitude must be evaluated on
                //   17 bits -- a [15:0] slice silently wraps 65536 -> 0.
                //----------------------------------------------------------
                ST_WDIV: begin
                    if (dv_done) begin
                        if (dvqr[16:0] > 17'd65536) begin
                            // out of range -> saturate to 1.0
                            if (rev_r ^ (chi != 3'd5))
                                u_flat[chi*32 +: 32] <= -32'sd65536;
                            else
                                u_flat[chi*32 +: 32] <= 32'sd65536;
                        end else begin
                            // Apply sign: rev XOR (chi != 5)
                            //   rev_r   = 1 for reverse (should be negative)
                            //   chi!=5  = 1 to negate (undo STM32 negation)
                            //   XOR: true -> negate the magnitude
                            if (rev_r ^ (chi != 3'd5))
                                u_flat[chi*32 +: 32] <= -$signed({15'd0, dvqr[16:0]});
                            else
                                u_flat[chi*32 +: 32] <= $signed({15'd0, dvqr[16:0]});
                        end
                        st <= ST_SIGN;
                    end
                end

                //----------------------------------------------------------
                // ST_SIGN: advance to next channel or finish
                //----------------------------------------------------------
                ST_SIGN: begin
                    if (chi == 3'd7) begin
                        done <= 1'b1;
                        st   <= ST_IDLE;
                    end else begin
                        chi <= chi + 3'd1;
                        st  <= ST_DIR;
                    end
                end

                default: st <= ST_IDLE;

            endcase
        end
    end

endmodule
