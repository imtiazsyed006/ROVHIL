//----------------------------------------------------------------------
// q16_mul.v -- Single shared Q16.16 signed multiply, fully pipelined
//
//   p = round(a * b / 2^16) = (a*b + 32768)[47:16]   (same as former fmul)
//
// Architecture (closes 100 MHz on xc7z020-1 without multicycle):
//   Sign-magnitude + four registered 16x16 partial products + registered
//   adder tree.  Every FF→FF path is at most one 16x16 mul or a shallow add.
//
// Latency from start pulse to valid pulse: 4 clocks.
//   c0 (on start): latch |a|,|b|,sign → st=1
//   c1: four 16x16 partial products
//   c2: sum partials → 64-bit unsigned product
//   c3: apply sign, +round, extract Q16.16; pulse valid
//----------------------------------------------------------------------
module q16_mul (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,   // 1-cycle pulse; ignored while busy
    input  wire signed [31:0] a,
    input  wire signed [31:0] b,
    output reg  signed [31:0] p,
    output reg                valid,   // 1-cycle when p is fresh
    output wire               busy
);
    reg [2:0] st;
    assign busy = (st != 3'd0);

    reg        sign_r;
    reg [31:0] ua_r, ub_r;
    reg [31:0] ppll, pplh, pphl, pphh;
    reg [63:0] prod_u;

    wire [15:0] a_lo = ua_r[15:0];
    wire [15:0] a_hi = ua_r[31:16];
    wire [15:0] b_lo = ub_r[15:0];
    wire [15:0] b_hi = ub_r[31:16];

    // Combinational helpers used only inside the registered stage-3 path
    // (operands already registered).
    wire [63:0] prod_s_w = sign_r ? (~prod_u + 64'd1) : prod_u;
    wire signed [63:0] rounded_w = $signed(prod_s_w) + 64'sd32768;

    always @(posedge clk) begin
        if (rst) begin
            st     <= 3'd0;
            valid  <= 1'b0;
            p      <= 32'sd0;
            sign_r <= 1'b0;
            ua_r   <= 32'd0;
            ub_r   <= 32'd0;
            ppll   <= 32'd0;
            pplh   <= 32'd0;
            pphl   <= 32'd0;
            pphh   <= 32'd0;
            prod_u <= 64'd0;
        end else begin
            valid <= 1'b0;
            case (st)
                3'd0: begin
                    if (start) begin
                        sign_r <= a[31] ^ b[31];
                        if (a[31])
                            ua_r <= (a == 32'h80000000) ? 32'h7FFFFFFF : (~a + 32'd1);
                        else
                            ua_r <= a;
                        if (b[31])
                            ub_r <= (b == 32'h80000000) ? 32'h7FFFFFFF : (~b + 32'd1);
                        else
                            ub_r <= b;
                        st <= 3'd1;
                    end
                end

                3'd1: begin
                    ppll <= a_lo * b_lo;
                    pplh <= a_lo * b_hi;
                    pphl <= a_hi * b_lo;
                    pphh <= a_hi * b_hi;
                    st   <= 3'd2;
                end

                3'd2: begin
                    prod_u <= {32'd0, ppll}
                            + ({16'd0, pplh, 16'd0})
                            + ({16'd0, pphl, 16'd0})
                            + ({pphh, 32'd0});
                    st <= 3'd3;
                end

                3'd3: begin
                    p     <= rounded_w[47:16];
                    valid <= 1'b1;
                    st    <= 3'd0;
                end

                default: st <= 3'd0;
            endcase
        end
    end
endmodule
