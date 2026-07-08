//----------------------------------------------------------------------
// pwm_measure.v -- 8-channel PWM duty-cycle measurement
//
// Measures the high-time and period of up to 8 PWM inputs (nominal 400 Hz,
// 44%..76% duty) using the 100 MHz clock as a timebase.
//
// Per channel, on every completed period it outputs:
//   * high_counts  : number of 100 MHz ticks the input was high
//   * total_counts : number of 100 MHz ticks in the full period
//   * active       : 1 if a rising edge was seen recently (signal present)
// Downstream (force_lut.v) treats inactive channels as zero force, so a
// disconnected/floating pin produces NO torque instead of a spurious one.
//
// Design notes:
//   * 2-FF synchronizer on each async input (metastability protection).
//   * Edge detection on the synchronized signal.
//   * 18-bit free-running counter (max 262143 > 250000 counts/period).
//   * Per-channel watchdog: if no rising edge within TIMEOUT ticks
//     (~5 ms > one 400 Hz period), the channel is declared inactive and its
//     high_counts is forced to 0 so force_lut -> zero force (not -3.52 kgf).
//
// Ports use FLAT buses for maximum portability.
//
// Latency: first valid sample ~1 period (2.5 ms) after a real signal appears.
//----------------------------------------------------------------------
module pwm_measure #(
    parameter NCH = 8,
    parameter CW  = 18,            // counter width
    // watchdog timeout: 5 ms @ 100 MHz = 500000 ticks (> one 400Hz period).
    // 19 bits holds up to 524287.
    parameter WD  = 19
) (
    input  wire              clk,
    input  wire              rst,
    input  wire [NCH-1:0]    pwm_in,                 // raw async PWM inputs

    // captured counts, packed: channel i occupies bits [i*CW +: CW]
    output reg  [NCH*CW-1:0] high_counts_flat,
    output reg  [NCH*CW-1:0] total_counts_flat,
    output reg  [NCH-1:0]    channel_active,         // 1 = signal present
    output reg               valid                   // 1-cycle pulse on update
);

    integer i;

    // per-channel synchronizer + edge detect
    reg [NCH-1:0] sync0, sync1, sync2;

    // per-channel free-running period counter + armed flag + watchdog
    reg [CW-1:0] cnt  [0:NCH-1];
    reg          armed[0:NCH-1];
    reg [WD-1:0] wdog [0:NCH-1];        // counts up since last rising edge

    localparam [WD-1:0] WD_TIMEOUT = 19'd500_000;   // ~5 ms

    always @(posedge clk) begin
        if (rst) begin
            sync0 <= {NCH{1'b0}};
            sync1 <= {NCH{1'b0}};
            sync2 <= {NCH{1'b0}};
            valid <= 1'b0;
            high_counts_flat  <= {NCH*CW{1'b0}};
            total_counts_flat <= {NCH*CW{1'b0}};
            channel_active    <= {NCH{1'b0}};
            for (i = 0; i < NCH; i = i + 1) begin
                cnt[i]   <= {CW{1'b0}};
                armed[i] <= 1'b0;
                wdog[i]  <= {WD{1'b0}};
            end
        end else begin
            valid <= 1'b0;
            sync0 <= pwm_in;
            sync1 <= sync0;     // synchronized value (metastability-resolved)
            sync2 <= sync1;     // delayed for edge detection

            for (i = 0; i < NCH; i = i + 1) begin
                // ---- watchdog: counts time since last rising edge ----
                // any rising edge resets it; if it reaches the timeout the
                // channel is declared inactive.
                if (sync1[i] && !sync2[i]) begin
                    wdog[i] <= {WD{1'b0}};            // reset on rising edge
                end else if (wdog[i] != {WD{1'b1}}) begin
                    wdog[i] <= wdog[i] + 1'b1;        // tick up (saturate)
                end
                // activity flag: active iff watchdog < timeout
                channel_active[i] <= (wdog[i] < WD_TIMEOUT);

                // ---- period measurement ----
                // rising edge: start a new period
                if (sync1[i] && !sync2[i]) begin
                    if (armed[i]) begin
                        // period complete: publish total
                        total_counts_flat[i*CW +: CW] <= cnt[i];
                        valid <= 1'b1;
                    end
                    armed[i] <= 1'b1;
                    cnt[i]   <= {{(CW-1){1'b0}}, 1'b1};
                end
                // falling edge: latch high time
                else if (!sync1[i] && sync2[i] && armed[i]) begin
                    high_counts_flat[i*CW +: CW] <= cnt[i];
                end
                // running: increment counter (saturate to avoid rollover wrap)
                else if (armed[i] && cnt[i] != {CW{1'b1}}) begin
                    cnt[i] <= cnt[i] + 1'b1;
                end

                // ---- safety: inactive channel -> force high_counts to 0 ----
                // so downstream never computes a nonzero force from a dead pin.
                if (wdog[i] >= WD_TIMEOUT) begin
                    high_counts_flat[i*CW +: CW] <= {CW{1'b0}};
                end
            end
        end
    end

endmodule
