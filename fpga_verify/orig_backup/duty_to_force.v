//----------------------------------------------------------------------
// duty_to_force.v — Convert 8 CCR-scaled duty values → 8 normalized forces
//
// Implements the exact MATLAB map_pwm_to_moments per-channel algorithm:
//   1. pwm_us = duty >> 2, clamp [1100, 1900]
//   2. Forward  (pwm_us > 1500):  u_sat = +force / max_fwd_force
//      Reverse  (pwm_us < 1500):  u_sat = -force_mag / max_rev_force
//      Deadzone (pwm_us = 1500):  u_sat = 0
//   3. Clamp u_sat magnitude to [0, 65536]  (= 1.0 in Q16.16)
//   4. Per-channel negation undo: if chi != 5, u_sat = -u_sat
//      (MATLAB: ch6 stays, all others get negated)
//
// Uses dual thrust-curve LUTs matching MATLAB thruster_tables():
//   fwd_lut.mem — 92 entries, PWM 1536:4:1900, FWD_FORCE_KGF
//   rev_lut.mem — 92 entries, PWM 1100:4:1464, REV_FORCE_MAG_KGF
//
// Output: 8 × 32-bit signed Q16.16 normalized forces, packed [255:0].
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
    // Shared divider — 48-cycle shift-subtract
    //   quotient = (dividend_q16 << 16) / divisor_q16
    //==================================================================
    reg        dvs;           // 1-cycle pulse to launch
    reg        dvb;           // divider busy
    reg [5:0]  dvc;           // iteration count 0..48
    reg [47:0] dv_div;        // dividend shift register
    reg [47:0] dv_rem;        // running remainder
    reg [47:0] dvqr;          // running quotient
    reg [31:0] dvden;         // divisor (max_force)
    wire       dv_done = dvb && (dvc == 6'd48);
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
                end else begin
                    dvb <= 1'b0;
                end
            end
        end
    end

    //==================================================================
    // FSM states
    //==================================================================
    localparam [2:0]
        ST_IDLE   = 3'd0,
        ST_DIR    = 3'd1,   // compute pwm_us, direction, idx, frac
        ST_READ0  = 3'd2,   // read LUT[idx]
        ST_READ1  = 3'd3,   // read LUT[idx+1]
        ST_INTERP = 3'd4,   // force = f0 + frac*(f1-f0)/4
        ST_DIV    = 3'd5,   // launch divider
        ST_WDIV   = 3'd6,   // wait for divider
        ST_SIGN   = 3'd7;   // apply sign + per-channel negation, then next/done

    reg [2:0]  st;
    reg [2:0]  chi;

    // Per-channel registers
    reg        fwd_r;         // 1=forward, 0=reverse
    reg        dead_r;        // 1=deadzone or inactive
    reg        rev_r;         // 1=reverse → negative u_sat
    reg [31:0] maxf_r;        // FP_MAX_FWD_FORCE or FP_MAX_REV_FORCE
    reg [7:0]  idx_r;         // LUT base index
    reg [7:0]  idx1_r;        // idx+1, clamped
    reg [1:0]  frac_r;        // interpolation fraction (0..3)
    reg signed [31:0] f0_r, f1_r;  // LUT read values
    reg [31:0] dvdividend;     // force for divider
    reg [31:0] dvdivisor;      // max_force for divider

    //==================================================================
    // Combinational: pwm_us from duty input
    //==================================================================
    wire [11:0] pwm_us_w = duty_flat[chi*16+2 +: 12];  // duty >> 2

    // Clamped pwm_us
    wire [11:0] pwm_clamp =
        (pwm_us_w < 12'd1100) ? 12'd1100 :
        (pwm_us_w > 12'd1900) ? 12'd1900 :
                                 pwm_us_w;

    // Forward offset: pwm_us - 1536, clamped to 0..364
    wire [11:0] fwd_off = (pwm_clamp < `FWD_PWM_START) ? 12'd0 :
                           (pwm_clamp - `FWD_PWM_START);
    // Reverse offset: pwm_us - 1100, clamped to 0..364
    wire [11:0] rev_off = (pwm_clamp > `REV_PWM_END) ?
                           (`REV_PWM_END - `REV_PWM_START) :
                           (pwm_clamp - `REV_PWM_START);

    // Forward indices (0..91, idx+1 capped at 90)
    wire [7:0] fwd_idx_w  = fwd_off >> 2;
    wire [7:0] fwd_idx1_w = ((fwd_off >> 2) >= 8'd90) ? 8'd90 : ((fwd_off >> 2) + 8'd1);
    wire [1:0] fwd_frac_w = fwd_off[1:0];

    // Reverse indices (0..91, idx+1 capped at 90)
    wire [7:0] rev_idx_w  = rev_off >> 2;
    wire [7:0] rev_idx1_w = ((rev_off >> 2) >= 8'd90) ? 8'd90 : ((rev_off >> 2) + 8'd1);
    wire [1:0] rev_frac_w = rev_off[1:0];

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
                //   Uses pwm_clamp (combinational) so everything is valid
                //   this cycle and registered for use in ST_READ0.
                //----------------------------------------------------------
                ST_DIR: begin
                    // Dead band: ±1 μs around 1500 handles integer truncation
                    // of duty>>2 (e.g. 59.99%→1499, 60.00%→1500, 60.01%→1501)
                    if (!ch_active[chi] || (pwm_clamp >= 12'd1499 && pwm_clamp <= 12'd1501)) begin
                        // Dead zone or inactive → u_sat = 0
                        dead_r <= 1'b1;
                        fwd_r  <= 1'b0;
                        rev_r  <= 1'b0;
                    end else if (pwm_clamp > 12'd1501) begin
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
                    st   <= ST_INTERP;
                end

                //----------------------------------------------------------
                // ST_INTERP: force = f0 + frac*(f1-f0)/4
                //----------------------------------------------------------
                ST_INTERP: begin
                    if (dead_r) begin
                        dvdividend <= 32'd0;
                    end else begin
                        dvdividend <= f0_r + (($signed({1'b0, frac_r}) *
                                            (f1_r - f0_r)) >>> 2);
                    end
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
                        // force=0 → u_sat=0
                        u_flat[chi*32 +: 32] <= 32'sd0;
                        st <= ST_SIGN;
                    end else begin
                        dvs <= 1'b1;
                        st  <= ST_WDIV;
                    end
                end

                //----------------------------------------------------------
                // ST_WDIV: wait for divider, then apply sign
                //----------------------------------------------------------
                ST_WDIV: begin
                    if (dv_done) begin
                        // dvqr[15:0] = u_sat magnitude (Q16.16, 0..65536)
                        // Clamp to [0, 65536]
                        if (dvqr[15:0] > 16'd65535) begin
                            // overflow → saturate to 1.0
                            if (rev_r ^ (chi != 3'd5))
                                u_flat[chi*32 +: 32] <= -32'sd65536;
                            else
                                u_flat[chi*32 +: 32] <= 32'sd65536;
                        end else begin
                            // Apply sign: rev XOR (chi != 5)
                            //   rev_r   = 1 for reverse (should be negative)
                            //   chi!=5  = 1 to negate (undo STM32 negation)
                            //   XOR: true → negate the magnitude
                            if (rev_r ^ (chi != 3'd5))
                                u_flat[chi*32 +: 32] <= -$signed({16'd0, dvqr[15:0]});
                            else
                                u_flat[chi*32 +: 32] <= $signed({16'd0, dvqr[15:0]});
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
