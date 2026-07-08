module uart #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD        = 115200
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        rx,
    output reg         tx,

    input  wire        par_en,

    input  wire        tx_req,
    output reg         tx_end,
    input  wire [7:0]  tx_data,

    output reg         rx_ready,
    output reg  [7:0]  rx_data,

    output wire        tx_busy     // <<< ADDED
);

    localparam UART_IDLE  = 1'b1;
    localparam UART_START = 1'b0;

    localparam integer DIV_MAX = (CLK_FREQ_HZ / BAUD) - 1;

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_DATA  = 3'd1;
    localparam [2:0] S_PAR   = 3'd2;
    localparam [2:0] S_STOP1 = 3'd3;
    localparam [2:0] S_STOP2 = 3'd4;

    // TX busy = not idle
    reg [2:0] tx_fsm;
    assign tx_busy = (tx_fsm != S_IDLE);   // <<< ADDED

    // ----------------------------
    // TX baud enable
    // ----------------------------
    reg [$clog2(DIV_MAX+1)-1:0] tx_div;
    reg tx_clk_en;

    always @(posedge clk) begin
        if (rst) begin
            tx_div    <= 0;
            tx_clk_en <= 1'b0;
        end else begin
            if (tx_div == DIV_MAX[$clog2(DIV_MAX+1)-1:0]) begin
                tx_div    <= 0;
                tx_clk_en <= 1'b1;
            end else begin
                tx_div    <= tx_div + 1'b1;
                tx_clk_en <= 1'b0;
            end
        end
    end

    // ----------------------------
    // TX process
    // ----------------------------
    reg [7:0] tx_data_tmp;
    reg [2:0] tx_data_cnt;
    reg       tx_par_bit;

    always @(posedge clk) begin
        if (rst) begin
            tx         <= UART_IDLE;
            tx_end     <= 1'b0;
            tx_fsm     <= S_IDLE;
            tx_data_tmp<= 8'h00;
            tx_data_cnt<= 3'd0;
            tx_par_bit <= 1'b0;
        end else begin
            if (tx_clk_en) begin
                tx_end <= 1'b0;
                tx     <= UART_IDLE;

                case (tx_fsm)
                    S_IDLE: begin
                        if (tx_req) begin
                            tx          <= UART_START;
                            tx_data_tmp <= tx_data;
                            tx_fsm      <= S_DATA;
                            tx_data_cnt <= 3'b111;
                            tx_par_bit  <= 1'b0;
                        end
                    end

                    S_DATA: begin
                        tx         <= tx_data_tmp[0];
                        tx_par_bit <= tx_par_bit ^ tx_data_tmp[0];

                        if (tx_data_cnt == 3'd0) begin
                            if (par_en) tx_fsm <= S_PAR;
                            else        tx_fsm <= S_STOP1;
                            tx_data_cnt <= 3'b111;
                        end else begin
                            tx_data_tmp <= {1'b0, tx_data_tmp[7:1]};
                            tx_data_cnt <= tx_data_cnt - 1'b1;
                        end
                    end

                    S_PAR:   begin tx <= tx_par_bit; tx_fsm <= S_STOP1; end
                    S_STOP1: begin tx <= UART_IDLE;  tx_fsm <= S_STOP2; end

                    S_STOP2: begin
                        tx_end <= 1'b1;
                        tx     <= UART_IDLE;
                        tx_fsm <= S_IDLE;
                    end

                    default: tx_fsm <= S_IDLE;
                endcase
            end
        end
    end

    // ----------------------------
    // RX side unchanged (your code)
    // ----------------------------
    // ... keep your RX debounce/start/clk_gen/rx_proc exactly as-is ...
    // (omitted here for brevity in this snippet, but you can paste your RX code below unchanged)
    // ----------------------------
    // RX debounce (4-sample shift)
    // ----------------------------
    reg [3:0] deb_buf;
    reg       rx_data_deb;

    always @(posedge clk) begin
        if (rst) begin
            deb_buf     <= 4'h0;
            rx_data_deb <= 1'b1;
        end else begin
            deb_buf <= {deb_buf[2:0], rx};

            if (deb_buf == 4'b0000)      rx_data_deb <= 1'b0;
            else if (deb_buf == 4'b1111) rx_data_deb <= 1'b1;
        end
    end

    // ----------------------------
    // RX start detect (falling edge)
    // ----------------------------
    reg [2:0] rx_fsm;      // declared here: used by start-detect below
    reg rx_data_old;
    reg rx_rcv_init;

    always @(posedge clk) begin
        if (rst) begin
            rx_data_old <= 1'b0;
            rx_rcv_init <= 1'b0;
        end else begin
            rx_rcv_init <= (rx_data_old == 1'b1) && (rx_data_deb == 1'b0) && (rx_fsm == S_IDLE);
            rx_data_old <= rx_data_deb;
        end
    end

    // ----------------------------
    // RX baud enable (resets on start detect like original)
    // ----------------------------
    reg [$clog2(DIV_MAX+1)-1:0] rx_div;
    reg rx_clk_en;

    always @(posedge clk) begin
        if (rst) begin
            rx_div    <= 0;
            rx_clk_en <= 1'b0;
        end else begin
            if (rx_rcv_init || (rx_div == DIV_MAX[$clog2(DIV_MAX+1)-1:0])) begin
                rx_div    <= 0;
                rx_clk_en <= 1'b1;
            end else begin
                rx_div    <= rx_div + 1'b1;
                rx_clk_en <= 1'b0;
            end
        end
    end

    // ----------------------------
    // RX process
    // ----------------------------
    reg [7:0] rx_data_tmp;
    reg [2:0] rx_data_cnt;
    reg       rx_par_bit;

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            rx_fsm      <= S_IDLE;
            rx_ready    <= 1'b0;
            rx_data     <= 8'h00;
            rx_data_tmp <= 8'h00;
            rx_data_cnt <= 3'd0;
            rx_par_bit  <= 1'b0;
        end else begin
            rx_ready <= 1'b0;

            if (rx_clk_en) begin
                case (rx_fsm)
                    S_IDLE: begin
                        if (rx_data_deb == UART_START) begin
                            rx_fsm <= S_DATA;
                        end
                        rx_par_bit  <= 1'b0;
                        rx_data_cnt <= 3'd0;
                    end

                    S_DATA: begin
                        if (par_en) rx_par_bit <= rx_par_bit ^ rx; // parity uses raw rx in original

                        if (rx_data_cnt == 3'd7) begin
                            // build rx_data like the VHDL did
                            rx_data[7] <= rx;
                            for (i = 0; i <= 6; i = i + 1) begin
                                rx_data[i] <= rx_data_tmp[6 - i];
                            end

                            if (par_en) begin
                                rx_fsm <= S_PAR;
                            end else begin
                                rx_ready <= 1'b1;
                                rx_fsm   <= S_IDLE;
                            end
                        end else begin
                            rx_data_tmp <= {rx_data_tmp[6:0], rx};
                            rx_data_cnt <= rx_data_cnt + 1'b1;
                        end
                    end

                    S_PAR: begin
                        rx_fsm <= S_IDLE;
                        if (rx_par_bit == rx) begin
                            rx_ready <= 1'b1;
                        end
                    end

                    default: rx_fsm <= S_IDLE;
                endcase
            end
        end
    end
endmodule
