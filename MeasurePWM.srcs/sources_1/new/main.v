`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// main.v -- 8-ch PWM → duty → force → moments → ROV model → UM7 UART output
//
// Pipeline (triggered every 20 ms):
//   pwm_measure   continuous capture of 8 PWM inputs (100 MHz ticks)
//   duty_calc     (hi * 10000) / tot  → 8 duty_scaled values (0..10000)
//   duty_to_force duty/4 → LUT       → 8 forces (Q16.16 kgf)
//   tau_alloc     B * u              → Mx, My, Mz (Q16.16 N*m)
//   rov_model     RK4 6-DOF          → true roll,pitch,yaw,p,q,r (Q16.16)
//   sensor_model  AHRS corruption    → measured RPY + pqr (rov_sensor.py)
//   um7_tx        UM7 binary packets → UART @ 115200 baud
//                 (quaternion is built from *measured* RPY)
//
// UART output: UM7 "snp" binary protocol (quaternion + gyro packets)
//   Packet 1: 'snp' + quaternion (int16 w,x,y,z scaled by 29789.09)
//   Packet 2: 'snp' + gyro (float32 p,q,r deg/s)
//   Total: 34 bytes, ~2.9 ms at 115200 baud
//
// LEDs: LD0=heartbeat, LD1-3=Mx/My/Mz present, LD4-6=Mx/My/Mz sign, LD7=any PWM
//////////////////////////////////////////////////////////////////////////////////
`include "rov_params.vh"

module main (
    input  wire        clk_100m,
    input  wire        uart_rx,
    output wire        uart_tx,
    input  wire [7:0]  pwm_in,
    output reg  [7:0]  led,
    // Debug UART (JA3=TX, JA4=RX) — outputs Mx, My, Mz torques
    input  wire        dbg_uart_rx,
    output wire        dbg_uart_tx
);

    //------------------------------------------------------------------
    // Power-on reset (~10 ms @ 100 MHz)
    //------------------------------------------------------------------
    reg [19:0] por_cnt = 20'd0;
    reg        rst     = 1'b1;
    always @(posedge clk_100m) begin
        if (rst) begin
            if (por_cnt == 20'd1_000_000) rst <= 1'b0;
            else por_cnt <= por_cnt + 20'd1;
        end
    end

    //==================================================================
    // 1. PWM MEASUREMENT (continuous, 8 channels)
    //==================================================================
    localparam NCH = 8;
    localparam CW  = 18;

    wire [NCH*CW-1:0] hi_cnt_flat;
    wire [NCH*CW-1:0] tot_cnt_flat;
    wire [NCH-1:0]    ch_active;
    wire              pwm_valid;

    pwm_measure #(.NCH(NCH), .CW(CW)) u_pwm_meas (
        .clk              (clk_100m),
        .rst              (rst),
        .pwm_in           (pwm_in),
        .high_counts_flat (hi_cnt_flat),
        .total_counts_flat(tot_cnt_flat),
        .channel_active   (ch_active),
        .valid            (pwm_valid)
    );

    //==================================================================
    // 2. 20 ms TIMER — triggers the entire pipeline
    //==================================================================
    reg [20:0] step_timer;
    reg        tick_002s;

    always @(posedge clk_100m) begin
        if (rst) begin
            step_timer <= 21'd0;
            tick_002s  <= 1'b0;
        end else begin
            tick_002s <= 1'b0;
            if (step_timer == 21'd1_999_999) begin
                step_timer <= 21'd0;
                tick_002s  <= 1'b1;
            end else begin
                step_timer <= step_timer + 21'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // Capture raw counts on tick
    //------------------------------------------------------------------
    reg [NCH*CW-1:0] cap_hi, cap_tot;
    reg              cap_valid;

    always @(posedge clk_100m) begin
        if (rst) begin
            cap_valid <= 1'b0;
            cap_hi    <= {NCH*CW{1'b0}};
            cap_tot   <= {NCH*CW{1'b0}};
        end else begin
            cap_valid <= 1'b0;
            if (tick_002s) begin
                cap_hi    <= hi_cnt_flat;
                cap_tot   <= tot_cnt_flat;
                cap_valid <= 1'b1;
            end
        end
    end

    //==================================================================
    // 3. DUTY COMPUTATION — sequential (hi*10000)/tot per channel
    //    Same sequential divider as before. ~280 cycles ≈ 2.8 µs.
    //==================================================================
    reg [15:0] duty_val [0:NCH-1];
    reg        duty_ready;

    // packed alias for module port connection
    wire [127:0] duty_flat;
    genvar gi;
    generate
        for (gi = 0; gi < NCH; gi = gi + 1) begin : gen_duty_pack
            assign duty_flat[gi*16 +: 16] = duty_val[gi];
        end
    endgenerate

    localparam C_IDLE     = 3'd0;
    localparam C_MULT     = 3'd1;
    localparam C_DIV_INIT = 3'd2;
    localparam C_DIV_LOOP = 3'd3;
    localparam C_STORE    = 3'd4;

    reg [2:0]  c_st;
    reg [2:0]  c_ch;
    reg [31:0] c_hi, c_tot;
    reg [31:0] c_product, c_divisor;
    reg [31:0] c_remainder, c_quotient;
    reg [5:0]  c_bit;
    integer    jj;   // loop variable

    always @(posedge clk_100m) begin
        if (rst) begin
            c_st        <= C_IDLE;
            c_ch        <= 3'd0;
            c_hi        <= 32'd0;
            c_tot       <= 32'd0;
            c_product   <= 32'd0;
            c_divisor   <= 32'd0;
            c_remainder <= 32'd0;
            c_quotient  <= 32'd0;
            c_bit       <= 6'd0;
            duty_ready  <= 1'b0;
            for (jj = 0; jj < NCH; jj = jj + 1)
                duty_val[jj] <= 16'd0;
        end else begin
            duty_ready <= 1'b0;

            case (c_st)
                C_IDLE: begin
                    if (cap_valid) begin
                        c_ch  <= 3'd0;
                        c_hi  <= {14'd0, cap_hi[0*CW +: CW]};
                        c_tot <= {14'd0, cap_tot[0*CW +: CW]};
                        c_st  <= C_MULT;
                    end
                end

                C_MULT: begin
                    c_product <= c_hi * 32'd10000;
                    c_divisor <= c_tot;
                    if (c_tot == 32'd0) begin
                        c_quotient <= 32'd0;
                        c_st       <= C_STORE;
                    end else begin
                        c_st <= C_DIV_INIT;
                    end
                end

                C_DIV_INIT: begin
                    c_remainder <= 32'd0;
                    c_quotient  <= 32'd0;
                    c_bit       <= 6'd31;
                    c_st        <= C_DIV_LOOP;
                end

                C_DIV_LOOP: begin
                    if ({c_remainder[30:0], c_product[31]} >= c_divisor) begin
                        c_remainder <= {c_remainder[30:0], c_product[31]} - c_divisor;
                        c_quotient[c_bit] <= 1'b1;
                    end else begin
                        c_remainder <= {c_remainder[30:0], c_product[31]};
                    end
                    c_product <= {c_product[30:0], 1'b0};
                    if (c_bit == 6'd0)
                        c_st <= C_STORE;
                    else
                        c_bit <= c_bit - 6'd1;
                end

                C_STORE: begin
                    if (c_tot == 32'd0)
                        duty_val[c_ch] <= 16'd0;
                    else if (c_quotient > 32'd10000 || c_hi > c_tot)
                        duty_val[c_ch] <= 16'd10000;
                    else
                        duty_val[c_ch] <= c_quotient[15:0];

                    if (c_ch == 3'd7) begin
                        duty_ready <= 1'b1;
                        c_st       <= C_IDLE;
                    end else begin
                        c_ch  <= c_ch + 3'd1;
                        c_hi  <= {14'd0, cap_hi[(c_ch + 3'd1) * CW +: CW]};
                        c_tot <= {14'd0, cap_tot[(c_ch + 3'd1) * CW +: CW]};
                        c_st  <= C_MULT;
                    end
                end

                default: c_st <= C_IDLE;
            endcase
        end
    end

    //==================================================================
    // 4. DUTY → FORCE (sequential LUT, ~56 cycles ≈ 0.6 µs)
    //==================================================================
    wire        fl_done;
    wire [255:0] u_force_flat;

    duty_to_force u_d2f (
        .clk       (clk_100m),
        .rst       (rst),
        .start     (duty_ready),
        .done      (fl_done),
        .duty_flat (duty_flat),
        .ch_active (ch_active),
        .u_flat    (u_force_flat)
    );

    // Latch the force vector when duty_to_force finishes
    reg [255:0] u_lat_flat;
    always @(posedge clk_100m) begin
        if (rst)
            u_lat_flat <= 256'd0;
        else if (fl_done)
            u_lat_flat <= u_force_flat;
    end

    //==================================================================
    // 5. TAU ALLOC — u → f_N → B*f → Mx, My, Mz in N·m (3-cycle pipeline)
    //==================================================================
    wire signed [31:0] tau_mx, tau_my, tau_mz;

    tau_alloc u_tau (
        .clk   (clk_100m),
        .rst   (rst),
        .u_flat(u_lat_flat),
        .tau_mx(tau_mx),
        .tau_my(tau_my),
        .tau_mz(tau_mz)
    );

    // No extra sign flips here: the Python input signs [1,-1,-1] are applied
    // to the COMMANDS before allocation inside the STM32, so they are already
    // encoded in the measured PWM. B*f reconstructs the net wrench directly.
    wire [31:0] tau_roll  = tau_mx;
    wire [31:0] tau_pitch = tau_my;
    wire [31:0] tau_yaw   = tau_mz;

    //==================================================================
    // 6. ROV MODEL — RK4 6-DOF rotational dynamics (~1125+ cycles)
    //==================================================================
    // Delay fl_done by 5 to cover u_flat latch + tau_alloc's 3 stages
    reg [3:0] tau_delay_sr;
    always @(posedge clk_100m) begin
        if (rst)
            tau_delay_sr <= 4'd0;
        else
            tau_delay_sr <= {tau_delay_sr[2:0], fl_done};
    end

    reg model_start_r;
    always @(posedge clk_100m) begin
        if (rst)
            model_start_r <= 1'b0;
        else
            model_start_r <= tau_delay_sr[3];
    end

    wire        model_start = model_start_r;
    wire        model_done;
    wire [31:0] st_roll, st_pitch, st_yaw, st_p, st_q, st_r;

    // Python RK4 integrates each step from tau[k-1] to tau[k]
    // (attitude_pqr_step(y, tau_arr[i-1], tau_arr[i], ...)), so keep the
    // previous sample's torque and feed (tau0, tau1) = (prev, curr).
    reg signed [31:0] tau_prev_roll, tau_prev_pitch, tau_prev_yaw;
    reg signed [31:0] tau0_roll_h, tau0_pitch_h, tau0_yaw_h;
    reg signed [31:0] tau1_roll_h, tau1_pitch_h, tau1_yaw_h;

    always @(posedge clk_100m) begin
        if (rst) begin
            tau_prev_roll <= 0; tau_prev_pitch <= 0; tau_prev_yaw <= 0;
            tau0_roll_h   <= 0; tau0_pitch_h   <= 0; tau0_yaw_h   <= 0;
            tau1_roll_h   <= 0; tau1_pitch_h   <= 0; tau1_yaw_h   <= 0;
        end else if (model_start) begin
            tau0_roll_h    <= tau_prev_roll;
            tau0_pitch_h   <= tau_prev_pitch;
            tau0_yaw_h     <= tau_prev_yaw;
            tau1_roll_h    <= tau_roll;
            tau1_pitch_h   <= tau_pitch;
            tau1_yaw_h     <= tau_yaw;
            tau_prev_roll  <= tau_roll;
            tau_prev_pitch <= tau_pitch;
            tau_prev_yaw   <= tau_yaw;
        end
    end

    rov_model u_model (
        .clk       (clk_100m),
        .rst       (rst),
        .start     (model_start),
        .done      (model_done),
        .tau0_roll (tau0_roll_h), .tau0_pitch(tau0_pitch_h), .tau0_yaw(tau0_yaw_h),
        .tau1_roll (tau1_roll_h), .tau1_pitch(tau1_pitch_h), .tau1_yaw(tau1_yaw_h),
        .state_roll(st_roll),   .state_pitch(st_pitch),   .state_yaw(st_yaw),
        .state_p   (st_p),      .state_q   (st_q),        .state_r  (st_r),
        // debug taps (unused but must be connected)
        .dbg_st(), .dbg_kphase(),
        .dbg_rhs0(), .dbg_rhs1(), .dbg_rhs2(),
        .dbg_n0(),   .dbg_n1(),   .dbg_n2(),
        .dbg_det(),
        .dbg_Pd(),   .dbg_Qd(),   .dbg_Rd()
    );

    //==================================================================
    // 7. SENSOR MODEL
    //    PASSTHROUGH=1 (default): true plant state → UART (clean HIL).
    //    PASSTHROUGH=0: inject AHRS noise from rov_sensor.py constants.
    //    um7_tx builds the quaternion from RPY (STM32 never sees Euler
    //    on the wire — only quat + gyro packets).
    //==================================================================
    reg model_done_d;
    always @(posedge clk_100m) begin
        if (rst) model_done_d <= 1'b0;
        else     model_done_d <= model_done;
    end
    wire sensor_start = model_done && !model_done_d;

    wire        sensor_done;
    wire [31:0] meas_roll, meas_pitch, meas_yaw, meas_p, meas_q, meas_r;

    sensor_model #(
        .PASSTHROUGH(1)   // 1=clean rest@0; set 0 to enable AHRS noise
    ) u_sensor (
        .clk      (clk_100m),
        .rst      (rst),
        .start    (sensor_start),
        .done     (sensor_done),
        .roll_in  (st_roll),
        .pitch_in (st_pitch),
        .yaw_in   (st_yaw),
        .p_in     (st_p),
        .q_in     (st_q),
        .r_in     (st_r),
        .roll_out (meas_roll),
        .pitch_out(meas_pitch),
        .yaw_out  (meas_yaw),
        .p_out    (meas_p),
        .q_out    (meas_q),
        .r_out    (meas_r)
    );

    // Latch measured state; hold stable until um7 accepts them
    reg [31:0] cap_roll, cap_pitch, cap_yaw, cap_p, cap_q, cap_r;
    reg        cap_pending;   // 1 = new sample waiting for um7

    wire       um7_done;
    wire       um7_busy;
    wire       um7_send;
    wire [7:0] um7_data;
    // Never re-enter um7 while busy (prevents corrupted mid-packet samples
    // that showed up as intermittent 90–180° RPY glitches in Experiment.mat).
    wire       um7_start_w = cap_pending && !um7_busy;

    always @(posedge clk_100m) begin
        if (rst) begin
            cap_pending <= 1'b0;
            cap_roll  <= 0; cap_pitch <= 0; cap_yaw <= 0;
            cap_p     <= 0; cap_q     <= 0; cap_r   <= 0;
        end else begin
            if (sensor_done) begin
                cap_roll    <= meas_roll;
                cap_pitch   <= meas_pitch;
                cap_yaw     <= meas_yaw;
                cap_p       <= meas_p;
                cap_q       <= meas_q;
                cap_r       <= meas_r;
                cap_pending <= 1'b1;
            end else if (um7_start_w)
                cap_pending <= 1'b0;
        end
    end

    //==================================================================
    // 8. UM7 PACKET TRANSMITTER
    //==================================================================

    // byte-channel wires shared by um7_tx and the uart_tx_byte_if below.
    wire       byte_busy, byte_done, byte_send;
    wire [7:0] byte_data;

    um7_tx u_um7 (
        .clk      (clk_100m),
        .rst      (rst),
        .start    (um7_start_w),
        .done     (um7_done),
        .busy     (um7_busy),
        .roll     (cap_roll),
        .pitch    (cap_pitch),
        .yaw      (cap_yaw),
        .p        (cap_p),
        .q        (cap_q),
        .r        (cap_r),
        .send_byte(um7_send),
        .byte_out (um7_data),
        .byte_busy(byte_busy),
        .byte_done(byte_done)
    );

    //==================================================================
    // 9. UART + BYTE INTERFACE
    //==================================================================
    wire       tx_end, tx_busy, rx_ready;
    wire [7:0] rx_data;
    wire       tx_req_w;
    wire [7:0] tx_data_w;

    uart #(
        .CLK_FREQ_HZ(100_000_000),
        .BAUD       (115200)
    ) u_uart (
        .clk     (clk_100m),
        .rst     (rst),
        .rx      (uart_rx),
        .tx      (uart_tx),
        .par_en  (1'b0),
        .tx_req  (tx_req_w),
        .tx_end  (tx_end),
        .tx_data (tx_data_w),
        .rx_ready(rx_ready),
        .rx_data (rx_data),
        .tx_busy (tx_busy)
    );

    uart_tx_byte_if u_byteif (
        .clk     (clk_100m),
        .rst     (rst),
        .send    (byte_send),
        .data    (byte_data),
        .busy    (byte_busy),
        .done    (byte_done),
        .tx_req  (tx_req_w),
        .tx_data (tx_data_w),
        .tx_busy (tx_busy),
        .tx_end  (tx_end)
    );

    // byte mux: um7_tx drives the byte interface
    assign byte_send = um7_send;
    assign byte_data = um7_data;

    //==================================================================
    // 10. DEBUG UART — outputs 8 duties + 3 torques on JA3/JA4
    //    Format: D0..D7 Mx My Mz  (11 hex words, space-sep, CRLF)
    //==================================================================
    // ---- latch duty values and torques when tau becomes valid ----
    reg [127:0] dbg_duty_flat;    // 8 × 16-bit duties
    reg signed [31:0] dbg_mx, dbg_my, dbg_mz;
    reg               dbg_cap_valid;

    always @(posedge clk_100m) begin
        if (rst) begin
            dbg_duty_flat <= 128'd0;
            dbg_mx        <= 0;
            dbg_my        <= 0;
            dbg_mz        <= 0;
            dbg_cap_valid <= 1'b0;
        end else begin
            dbg_cap_valid <= 1'b0;
            if (model_start) begin            // tau_alloc output valid here
                dbg_duty_flat <= duty_flat;   // captured at same pipeline stage
                dbg_mx        <= tau_mx;
                dbg_my        <= tau_my;
                dbg_mz        <= tau_mz;
                dbg_cap_valid <= 1'b1;
            end
        end
    end

    // ---- debug UART IP ----
    wire       dbg_tx_end, dbg_tx_busy, dbg_rx_ready;
    wire [7:0] dbg_rx_data;
    wire       dbg_tx_req_w;
    wire [7:0] dbg_tx_data_w;

    uart #(
        .CLK_FREQ_HZ(100_000_000),
        .BAUD       (115200)
    ) u_dbg_uart (
        .clk     (clk_100m),
        .rst     (rst),
        .rx      (dbg_uart_rx),
        .tx      (dbg_uart_tx),
        .par_en  (1'b0),
        .tx_req  (dbg_tx_req_w),
        .tx_end  (dbg_tx_end),
        .tx_data (dbg_tx_data_w),
        .rx_ready(dbg_rx_ready),
        .rx_data (dbg_rx_data),
        .tx_busy (dbg_tx_busy)
    );

    // ---- debug byte interface ----
    wire       dbg_byte_busy, dbg_byte_done, dbg_byte_send;
    wire [7:0] dbg_byte_data;

    uart_tx_byte_if u_dbg_byteif (
        .clk     (clk_100m),
        .rst     (rst),
        .send    (dbg_byte_send),
        .data    (dbg_byte_data),
        .busy    (dbg_byte_busy),
        .done    (dbg_byte_done),
        .tx_req  (dbg_tx_req_w),
        .tx_data (dbg_tx_data_w),
        .tx_busy (dbg_tx_busy),
        .tx_end  (dbg_tx_end)
    );

    // ---- debug hex printer ----
    reg        dbg_hex_start;
    reg [31:0] dbg_hex_value;
    wire       dbg_hex_busy, dbg_hex_done;
    wire       dbg_hex_send;
    wire [7:0] dbg_hex_data;

    uart_print_hex u_dbg_hex (
        .clk       (clk_100m),
        .rst       (rst),
        .start     (dbg_hex_start),
        .value     (dbg_hex_value),
        .busy      (dbg_hex_busy),
        .done      (dbg_hex_done),
        .send_byte (dbg_hex_send),
        .byte_out  (dbg_hex_data),
        .byte_busy (dbg_byte_busy),
        .byte_done (dbg_byte_done)
    );

    // ---- debug UART sequencer: 11 values (D0..D7, Mx, My, Mz) + CRLF ----
    // val_idx 0..7 = duty[0..7] (16-bit, zero-extended to 32-bit)
    // val_idx 8..10 = Mx, My, Mz (signed Q16.16)
    localparam DBG_IDLE     = 3'd0;
    localparam DBG_HEX      = 3'd1;   // start hex printer for val_idx
    localparam DBG_WHEX     = 3'd2;   // wait hex done
    localparam DBG_SEP      = 3'd3;   // send separator (space / CR / LF)
    localparam DBG_WSEP     = 3'd4;   // wait separator done

    reg [2:0]  dbg_st;
    reg [3:0]  dbg_idx;          // 0..10
    reg        dbg_cr_sent;      // for CR→LF sequence after last value

    reg        dbg_byte_send_r;
    reg [7:0]  dbg_byte_data_r;

    assign dbg_byte_send = dbg_hex_busy ? dbg_hex_send   : dbg_byte_send_r;
    assign dbg_byte_data = dbg_hex_busy ? dbg_hex_data   : dbg_byte_data_r;

    // combinational: select the 32-bit value for the current index
    wire [15:0] dbg_duty_sel = dbg_duty_flat[dbg_idx[2:0]*16 +: 16];
    wire [31:0] dbg_val_sel;
    assign dbg_val_sel =
        (dbg_idx == 4'd8)  ? dbg_mx :
        (dbg_idx == 4'd9)  ? dbg_my :
        (dbg_idx == 4'd10) ? dbg_mz :
                             {16'd0, dbg_duty_sel};

    always @(posedge clk_100m) begin
        if (rst) begin
            dbg_st         <= DBG_IDLE;
            dbg_idx        <= 4'd0;
            dbg_cr_sent    <= 1'b0;
            dbg_hex_start  <= 1'b0;
            dbg_hex_value  <= 32'd0;
            dbg_byte_send_r<= 1'b0;
            dbg_byte_data_r<= 8'd0;
        end else begin
            dbg_hex_start   <= 1'b0;
            dbg_byte_send_r <= 1'b0;

            case (dbg_st)
                DBG_IDLE: begin
                    dbg_idx     <= 4'd0;
                    dbg_cr_sent <= 1'b0;
                    if (dbg_cap_valid)
                        dbg_st <= DBG_HEX;
                end

                // start hex printer for current value
                DBG_HEX: begin
                    dbg_hex_value <= dbg_val_sel;
                    dbg_hex_start <= 1'b1;
                    dbg_st        <= DBG_WHEX;
                end
                DBG_WHEX: if (dbg_hex_done) dbg_st <= DBG_SEP;

                // send separator
                DBG_SEP: begin
                    if (dbg_idx == 4'd10) begin
                        // last value (Mz): CR then LF
                        if (!dbg_cr_sent) begin
                            dbg_byte_send_r <= 1'b1;
                            dbg_byte_data_r <= 8'h0D;   // CR
                        end else begin
                            dbg_byte_send_r <= 1'b1;
                            dbg_byte_data_r <= 8'h0A;   // LF
                        end
                    end else begin
                        dbg_byte_send_r <= 1'b1;
                        dbg_byte_data_r <= 8'h20;       // space
                    end
                    dbg_st <= DBG_WSEP;
                end

                // wait for separator byte to finish
                DBG_WSEP: begin
                    if (dbg_byte_done) begin
                        if (dbg_idx == 4'd10 && dbg_cr_sent) begin
                            // LF done → back to idle
                            dbg_st <= DBG_IDLE;
                        end else if (dbg_idx == 4'd10 && !dbg_cr_sent) begin
                            // CR done → send LF next
                            dbg_cr_sent <= 1'b1;
                            dbg_st      <= DBG_SEP;
                        end else begin
                            // advance to next value
                            dbg_idx <= dbg_idx + 4'd1;
                            dbg_st  <= DBG_HEX;
                        end
                    end
                end

                default: dbg_st <= DBG_IDLE;
            endcase
        end
    end

    //==================================================================
    // 10. LED INDICATORS
    // LD0: heartbeat (~1 Hz)
    // LD1-3: Mx/My/Mz present
    // LD4-6: Mx/My/Mz sign (positive)
    // LD7: any PWM channel active
    //==================================================================
    localparam signed [31:0] TAU_THRESH = 32'sd32768;   // 0.5 N*m (Q16.16)

    wire signed [31:0] mx_abs = tau_mx[31] ? (-tau_mx) : tau_mx;
    wire signed [31:0] my_abs = tau_my[31] ? (-tau_my) : tau_my;
    wire signed [31:0] mz_abs = tau_mz[31] ? (-tau_mz) : tau_mz;

    reg [25:0] hb_cnt;
    reg        hb_toggle;

    always @(posedge clk_100m) begin
        if (rst) begin
            hb_cnt    <= 26'd0;
            hb_toggle <= 1'b0;
            led       <= 8'd0;
        end else begin
            if (hb_cnt == 26'd49_999_999) begin
                hb_cnt    <= 26'd0;
                hb_toggle <= ~hb_toggle;
            end else begin
                hb_cnt <= hb_cnt + 26'd1;
            end

            led[0] <= hb_toggle;                            // heartbeat
            led[1] <= (mx_abs > TAU_THRESH);                // Mx present
            led[2] <= (my_abs > TAU_THRESH);                // My present
            led[3] <= (mz_abs > TAU_THRESH);                // Mz present
            led[4] <= (mx_abs > TAU_THRESH) & ~tau_mx[31];  // Mx +
            led[5] <= (my_abs > TAU_THRESH) & ~tau_my[31];  // My +
            led[6] <= (mz_abs > TAU_THRESH) & ~tau_mz[31];  // Mz +
            led[7] <= (| ch_active);                         // any PWM
        end
    end

endmodule
