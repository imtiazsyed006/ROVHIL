//----------------------------------------------------------------------
// uart_print_hex.v -- Sends a 32-bit value as 8 ASCII hex digits + space
// via the uart_tx_byte_if interface
//----------------------------------------------------------------------
module uart_print_hex (
    input  wire         clk,
    input  wire         rst,
    input  wire         start,          // pulse to start sending
    input  wire [31:0]  value,          // Q16.16 value to print
    output reg          busy,
    output reg          done,           // 1-cycle pulse when done

    // to byte-sender
    output reg          send_byte,
    output reg  [7:0]   byte_out,
    input  wire         byte_busy,
    input  wire         byte_done
);

    localparam ST_IDLE = 2'd0;
    localparam ST_SEND = 2'd1;
    localparam ST_WAIT = 2'd2;

    reg [1:0] st;
    reg [2:0] nibble;        // 0..7 (8 hex digits)
    reg [31:0] val_shift;    // shifting value

    function [7:0] nibble_to_ascii;
        input [3:0] nib;
        begin
            nibble_to_ascii = (nib <= 4'd9) ? (8'h30 + nib) : (8'h37 + nib); // '0'-'9' or 'A'-'F'
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            st        <= ST_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            send_byte <= 1'b0;
            byte_out  <= 8'd0;
            nibble    <= 3'd0;
            val_shift <= 32'd0;
        end else begin
            done      <= 1'b0;
            send_byte <= 1'b0;

            case (st)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        val_shift <= value;
                        nibble    <= 3'd7;  // start from MS nibble
                        st        <= ST_SEND;
                    end
                end

                ST_SEND: begin
                    if (!byte_busy) begin
                        // Send current nibble as ASCII
                        byte_out  <= nibble_to_ascii(val_shift[31:28]);
                        send_byte <= 1'b1;
                        st        <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (byte_done) begin
                        if (nibble == 3'd0) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            st   <= ST_IDLE;
                        end else begin
                            nibble    <= nibble - 3'd1;
                            val_shift <= {val_shift[27:0], 4'b0}; // shift left 4 bits
                            st        <= ST_SEND;
                        end
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule
