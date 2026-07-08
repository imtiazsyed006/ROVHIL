`timescale 1ns / 1ps
//----------------------------------------------------------------------
// tb_chain.v -- Co-simulation of duty -> force -> tau -> ROV model chain
// against the Python golden reference.
//
// Feeds duty_stim.hex (8 x 16-bit duty counts per sample, generated from
// Experiment.mat via the exact STM32 command->PWM pipeline) and writes
// per-sample results to tb_out.txt:
//   tau1_roll tau1_pitch tau1_yaw  phi theta psi p q r   (9 x Q16.16 hex)
//
// The glue between the modules is copied verbatim from main.v (fixed
// version) so the shipped structure is what gets verified.
//----------------------------------------------------------------------
module tb_chain;

    parameter NSAMP = 13374;

    reg clk = 1'b0;
    always #5 clk = ~clk;          // 100 MHz

    reg rst = 1'b1;

    //------------------------------------------------------------------
    // Stimulus memory
    //------------------------------------------------------------------
    reg [15:0] duty_mem [0:NSAMP*8-1];
    initial $readmemh("duty_stim.hex", duty_mem);

    reg  [127:0] duty_flat;
    wire [7:0]   ch_active = 8'hFF;
    reg          d2f_start;

    //------------------------------------------------------------------
    // DUT chain (same wiring as main.v)
    //------------------------------------------------------------------
    wire         fl_done;
    wire [255:0] u_force_flat;

    duty_to_force u_d2f (
        .clk       (clk),
        .rst       (rst),
        .start     (d2f_start),
        .done      (fl_done),
        .duty_flat (duty_flat),
        .ch_active (ch_active),
        .u_flat    (u_force_flat)
    );

    reg [255:0] u_lat_flat;
    always @(posedge clk) begin
        if (rst)
            u_lat_flat <= 256'd0;
        else if (fl_done)
            u_lat_flat <= u_force_flat;
    end

    wire signed [31:0] tau_mx, tau_my, tau_mz;

    tau_alloc u_tau (
        .clk   (clk),
        .rst   (rst),
        .u_flat(u_lat_flat),
        .tau_mx(tau_mx),
        .tau_my(tau_my),
        .tau_mz(tau_mz)
    );

    wire [31:0] tau_roll  = tau_mx;
    wire [31:0] tau_pitch = tau_my;
    wire [31:0] tau_yaw   = tau_mz;

    reg [3:0] tau_delay_sr;
    always @(posedge clk) begin
        if (rst)
            tau_delay_sr <= 4'd0;
        else
            tau_delay_sr <= {tau_delay_sr[2:0], fl_done};
    end

    reg model_start_r;
    always @(posedge clk) begin
        if (rst)
            model_start_r <= 1'b0;
        else
            model_start_r <= tau_delay_sr[3];
    end

    wire model_start = model_start_r;

    reg signed [31:0] tau_prev_roll, tau_prev_pitch, tau_prev_yaw;
    reg signed [31:0] tau0_roll_h, tau0_pitch_h, tau0_yaw_h;
    reg signed [31:0] tau1_roll_h, tau1_pitch_h, tau1_yaw_h;

    always @(posedge clk) begin
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

    wire        model_done;
    wire [31:0] st_roll, st_pitch, st_yaw, st_p, st_q, st_r;

    rov_model u_model (
        .clk       (clk),
        .rst       (rst),
        .start     (model_start),
        .done      (model_done),
        .tau0_roll (tau0_roll_h), .tau0_pitch(tau0_pitch_h), .tau0_yaw(tau0_yaw_h),
        .tau1_roll (tau1_roll_h), .tau1_pitch(tau1_pitch_h), .tau1_yaw(tau1_yaw_h),
        .state_roll(st_roll),   .state_pitch(st_pitch),   .state_yaw(st_yaw),
        .state_p   (st_p),      .state_q   (st_q),        .state_r  (st_r),
        .dbg_st(), .dbg_kphase(),
        .dbg_rhs0(), .dbg_rhs1(), .dbg_rhs2(),
        .dbg_n0(),   .dbg_n1(),   .dbg_n2(),
        .dbg_det(),
        .dbg_Pd(),   .dbg_Qd(),   .dbg_Rd()
    );

    //------------------------------------------------------------------
    // Drive samples back-to-back (no 20 ms wait needed for the math)
    //------------------------------------------------------------------
    integer fh, k, j;

    initial begin
        fh = $fopen("tb_out.txt", "w");
        d2f_start = 1'b0;
        duty_flat = 128'd0;

        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        for (k = 0; k < NSAMP; k = k + 1) begin
            for (j = 0; j < 8; j = j + 1)
                duty_flat[j*16 +: 16] = duty_mem[k*8 + j];

            @(posedge clk);
            d2f_start <= 1'b1;
            @(posedge clk);
            d2f_start <= 1'b0;

            @(posedge clk);
            while (!model_done) @(posedge clk);

            $fdisplay(fh, "%08h %08h %08h %08h %08h %08h %08h %08h %08h",
                      tau1_roll_h, tau1_pitch_h, tau1_yaw_h,
                      st_roll, st_pitch, st_yaw, st_p, st_q, st_r);

            if (k % 1000 == 0)
                $display("sample %0d / %0d", k, NSAMP);
        end

        $fclose(fh);
        $display("TB DONE");
        $finish;
    end

endmodule
