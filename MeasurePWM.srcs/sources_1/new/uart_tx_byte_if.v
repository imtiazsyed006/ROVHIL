module uart_tx_byte_if (
    input  wire       clk,
    input  wire       rst,        // active-high
    // request from user logic
    input  wire       send,       // pulse to send 'data'
    input  wire [7:0] data,
    output reg        busy,       // 1 while this wrapper is sending a byte
    output reg        done,       // 1-cycle pulse when byte is fully sent

    // connection to your uart IP
    output reg        tx_req,
    output reg  [7:0] tx_data,
    input  wire       tx_busy,    // from uart (your added port)
    input  wire       tx_end
);

    localparam ST_IDLE = 2'd0;
    localparam ST_REQ  = 2'd1;
    localparam ST_WAIT = 2'd2;

    reg [1:0] st;

    always @(posedge clk) begin
        if (rst) begin
            st     <= ST_IDLE;
            tx_req <= 1'b0;
            tx_data<= 8'h00;
            busy   <= 1'b0;
            done   <= 1'b0;
        end else begin
            done <= 1'b0;

            case (st)
                ST_IDLE: begin
                    tx_req <= 1'b0;
                    busy   <= 1'b0;
                    if (send) begin
                        tx_data <= data;
                        st      <= ST_REQ;
                        busy    <= 1'b1;
                    end
                end

                ST_REQ: begin
                    // hold tx_req HIGH until uart accepts (tx_busy goes high)
                    tx_req <= 1'b1;
                    if (tx_busy) begin
                        tx_req <= 1'b0;
                        st     <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (tx_end) begin
                        done <= 1'b1;
                        st   <= ST_IDLE;
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule
