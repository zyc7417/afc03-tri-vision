// Timing candidate: register RAM read/pair sum/bin sum/accumulation/control.
// Same-frame global histogram equalization, explicitly TWO passes.
// External storage must replay exactly the same frame after lut_ready rises.
// No frame store, previous-frame LUT, or fixed curve is hidden inside this core.
`timescale 1ns/1ps
module histogram_equalize4 #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080,
    // Compile-time policy only; no CSR, CDC, frame identity or stream changes.
    // 0: identity diagnostic, 1: original equalization, 2: bounded enhancement.
    parameter integer HE_MODE = 2,
    parameter integer MAX_DELTA = 16
) (
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready,
    input wire [127:0] s_data, input wire [3:0] s_keep,
    input wire s_sof, input wire s_eol, input wire s_eof,
    output wire lut_ready,
    input wire r_valid, output wire r_ready,
    input wire [127:0] r_data, input wire [3:0] r_keep,
    input wire r_sof, input wire r_eol, input wire r_eof,
    output reg m_valid, input wire m_ready,
    output wire [127:0] m_data, output reg [3:0] m_keep,
    output reg m_sof, output reg m_eol, output reg m_eof,
    output reg protocol_error
);
    localparam integer PIXELS = WIDTH*HEIGHT;
    localparam integer WORDS = (WIDTH+3)/4;
    localparam integer TAIL = WIDTH-(WORDS-1)*4;
    localparam [3:0] TAIL_KEEP = (1<<TAIL)-1;
    localparam integer COUNT_BITS = $clog2(PIXELS+1) < 1 ? 1 : $clog2(PIXELS+1);
    localparam integer NUM_BITS = COUNT_BITS+8;
    localparam integer DIV_COUNT_BITS = $clog2(NUM_BITS+1);
    localparam [4:0] CLEAR=0, STATS=1, DRAIN_STATS=2,
                     SCAN_READ=3, SCAN_PAIR=4, SCAN_BIN=5, SCAN_ACCUM=6, SCAN_SUM=7,
                     BUILD_READ=8, BUILD_PAIR=9, BUILD_BIN=10, BUILD_ACCUM=11,
                     BUILD_CDF=12, BUILD_DELTA=13, BUILD_SCALE=14, BUILD_ROUND=15,
                     DIVIDE=16, ADVANCE=17, CONSTANT_LUT=18, REMAP=19,
                     DRAIN_OUTPUT=20, ERROR=21;
    reg [4:0] state;
    reg [7:0] bin_index;
    integer sx, sy, rx, ry;
    reg [COUNT_BITS-1:0] total_count, cdf_min, cdf, denominator;
    reg minimum_seen;
    reg [NUM_BITS-1:0] dividend, quotient;
    reg [COUNT_BITS:0] remainder;
    reg [DIV_COUNT_BITS-1:0] bits_left;
    reg [COUNT_BITS:0] pair01, pair23;
    reg [COUNT_BITS+1:0] bin_total_q, accumulated;
    reg [COUNT_BITS-1:0] delta_q;
    reg [NUM_BITS-1:0] scaled_q;

    wire stats_accept = s_valid && s_ready;
    wire replay_accept = r_valid && r_ready;
    wire stats_last = sx == WORDS-1 && sy == HEIGHT-1;
    wire replay_last = rx == WORDS-1 && ry == HEIGHT-1;
    assign s_ready = rst_n && !protocol_error && state == STATS;
    assign lut_ready = rst_n && !protocol_error && state == REMAP;
    assign r_ready = lut_ready && (!m_valid || m_ready);
    wire bad_stats = stats_accept &&
        ((s_keep != (sx == WORDS-1 ? TAIL_KEEP : 4'hf)) ||
         (s_sof != (sx == 0 && sy == 0)) || (s_eol != (sx == WORDS-1)) ||
         (s_eof != stats_last));
    wire bad_replay = replay_accept &&
        ((r_keep != (rx == WORDS-1 ? TAIL_KEEP : 4'hf)) ||
         (r_sof != (rx == 0 && ry == 0)) || (r_eol != (rx == WORDS-1)) ||
         (r_eof != replay_last));
    wire bad_input = bad_stats || bad_replay;
    wire scan_enable = state == SCAN_READ || state == BUILD_READ;
    wire [COUNT_BITS-1:0] lane_count [0:3];
    wire [3:0] pending;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin: hist_banks
        histogram_lane_bank #(.COUNT_BITS(COUNT_BITS)) bank (
            .clk(clk),.rst_n(rst_n),.abort(protocol_error || bad_input),
            .clear_enable(state == CLEAR),.clear_address(bin_index),
            .sample_valid(stats_accept && !bad_stats && s_keep[lane]),
            .sample_value(s_data[32*lane +: 8]),
            .scan_enable(scan_enable),.scan_address(bin_index),
            .scan_data(lane_count[lane]),.pending(pending[lane])
        );
    end endgenerate
    // No control signal is driven through the RAM-to-CDF arithmetic chain.
    // The extra two sum bits retain overflow detection; only checked totals
    // are truncated back to COUNT_BITS when committing the accumulator.
    wire [NUM_BITS-1:0] delta_extended = {{8{1'b0}},delta_q};
    wire [NUM_BITS-1:0] denominator_extended = {{8{1'b0}},denominator};
    wire [COUNT_BITS:0] shifted_remainder = {remainder[COUNT_BITS-1:0],dividend[NUM_BITS-1]};
    wire subtract_divisor = shifted_remainder >= {1'b0,denominator};
    wire [NUM_BITS-1:0] quotient_next = {quotient[NUM_BITS-2:0],subtract_divisor};

    // Four identical LUT copies provide four synchronous read ports. Only one
    // LUT entry is written per build cycle; no combinational four-port RAM.
    reg [7:0] lut0[0:255], lut1[0:255], lut2[0:255], lut3[0:255];
    reg [7:0] mapped0, mapped1, mapped2, mapped3;
    wire lut_write = rst_n && !protocol_error &&
        (state == CONSTANT_LUT ||
         (state == BUILD_CDF && accumulated <= {2'b0,cdf_min}) ||
         (state == DIVIDE && bits_left == 1));
    wire [7:0] equalized_value = state == CONSTANT_LUT ? bin_index :
                           (state == BUILD_CDF ? 8'd0 : quotient_next[7:0]);
    // Operates only while building the 256-entry LUT. Replay retains the same
    // four synchronous RAM reads per beat and the exact original handshake.
    // The convex mix and both clamp bounds are monotone in input intensity.
    // Thus a monotone HE LUT stays monotone; each output differs by <=MAX_DELTA.
    // This is bounded global enhancement, NOT CLAHE or sensor exposure control.
    function automatic [7:0] tone_value;
        input [7:0] original_value;
        input [7:0] he_value;
        reg [9:0] weighted;
        reg [7:0] mixed, lower_bound, upper_bound;
        begin
            weighted = {2'b0,original_value} +
                       ({2'b0,original_value} << 1) + {2'b0,he_value} + 10'd2;
            mixed = weighted[9:2];
            lower_bound = original_value < MAX_DELTA ? 8'd0 : original_value-MAX_DELTA;
            upper_bound = original_value > 255-MAX_DELTA ? 8'd255 : original_value+MAX_DELTA;
            if(HE_MODE == 0) tone_value = original_value;
            else if(HE_MODE == 1) tone_value = he_value;
            else if(mixed < lower_bound) tone_value = lower_bound;
            else if(mixed > upper_bound) tone_value = upper_bound;
            else tone_value = mixed;
        end
    endfunction
    initial begin
        if(HE_MODE < 0 || HE_MODE > 2 || MAX_DELTA < 0 || MAX_DELTA > 255)
            $fatal(1,"BAD_TONE_POLICY");
    end
    wire [7:0] lut_value = tone_value(bin_index,equalized_value);
    always @(posedge clk) begin
        if(lut_write) begin
            lut0[bin_index] <= lut_value; lut1[bin_index] <= lut_value;
            lut2[bin_index] <= lut_value; lut3[bin_index] <= lut_value;
        end
        if(replay_accept && !bad_replay) begin
            mapped0 <= lut0[r_data[0 +: 8]];
            mapped1 <= lut1[r_data[32 +: 8]];
            mapped2 <= lut2[r_data[64 +: 8]];
            mapped3 <= lut3[r_data[96 +: 8]];
        end
    end
    assign m_data[0 +: 32] = m_keep[0] ? {8'd0,mapped0,mapped0,mapped0} : 32'd0;
    assign m_data[32 +: 32] = m_keep[1] ? {8'd0,mapped1,mapped1,mapped1} : 32'd0;
    assign m_data[64 +: 32] = m_keep[2] ? {8'd0,mapped2,mapped2,mapped2} : 32'd0;
    assign m_data[96 +: 32] = m_keep[3] ? {8'd0,mapped3,mapped3,mapped3} : 32'd0;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= CLEAR; bin_index <= 0;
            sx <= 0; sy <= 0; rx <= 0; ry <= 0;
            total_count <= 0; cdf_min <= 0; cdf <= 0; denominator <= 0; minimum_seen <= 0;
            dividend <= 0; quotient <= 0; remainder <= 0; bits_left <= 0;
            pair01 <= 0; pair23 <= 0; bin_total_q <= 0; accumulated <= 0;
            delta_q <= 0; scaled_q <= 0;
            m_valid <= 0; m_keep <= 0; m_sof <= 0; m_eol <= 0; m_eof <= 0;
            protocol_error <= 0;
        end else if(bad_input) begin
            state <= ERROR; protocol_error <= 1; m_valid <= 0;
        end else if(!protocol_error) begin
            if(!m_valid || m_ready) begin
                m_valid <= replay_accept;
                if(replay_accept) begin
                    m_keep <= r_keep; m_sof <= r_sof; m_eol <= r_eol; m_eof <= r_eof;
                end
            end
            case(state)
                CLEAR: begin
                    if(bin_index == 255) begin
                        state <= STATS; bin_index <= 0;
                        sx <= 0; sy <= 0; rx <= 0; ry <= 0;
                        total_count <= 0; cdf_min <= 0; cdf <= 0; denominator <= 0;
                        minimum_seen <= 0;
                    end else bin_index <= bin_index + 1'b1;
                end
                STATS: if(stats_accept) begin
                    if(stats_last) state <= DRAIN_STATS;
                    else if(sx == WORDS-1) begin sx <= 0; sy <= sy+1; end
                    else sx <= sx+1;
                end
                DRAIN_STATS: if(pending == 0) begin state <= SCAN_READ; bin_index <= 0; end
                SCAN_READ: state <= SCAN_PAIR;
                SCAN_PAIR: begin
                    pair01 <= {1'b0,lane_count[0]} + {1'b0,lane_count[1]};
                    pair23 <= {1'b0,lane_count[2]} + {1'b0,lane_count[3]};
                    state <= SCAN_BIN;
                end
                SCAN_BIN: begin
                    bin_total_q <= {1'b0,pair01} + {1'b0,pair23};
                    state <= SCAN_ACCUM;
                end
                SCAN_ACCUM: begin
                    accumulated <= {2'b0,total_count} + bin_total_q;
                    state <= SCAN_SUM;
                end
                SCAN_SUM: begin
                    total_count <= accumulated[COUNT_BITS-1:0];
                    if(!minimum_seen && bin_total_q != 0) begin
                        cdf_min <= bin_total_q[COUNT_BITS-1:0]; minimum_seen <= 1;
                    end
                    if(accumulated > PIXELS) begin
                        state <= ERROR; protocol_error <= 1; m_valid <= 0;
                    end else if(bin_index == 255) begin
                        if(accumulated != PIXELS) begin
                            state <= ERROR; protocol_error <= 1; m_valid <= 0;
                        end else begin
                            // If the first occupied bin is 255, it is a constant frame.
                            if(!minimum_seen || cdf_min == PIXELS) state <= CONSTANT_LUT;
                            else begin state <= BUILD_READ; denominator <= PIXELS-cdf_min; end
                            bin_index <= 0; cdf <= 0;
                        end
                    end else begin bin_index <= bin_index + 1'b1; state <= SCAN_READ; end
                end
                CONSTANT_LUT: begin
                    if(bin_index == 255) begin state <= REMAP; rx <= 0; ry <= 0; end
                    else bin_index <= bin_index + 1'b1;
                end
                BUILD_READ: state <= BUILD_PAIR;
                BUILD_PAIR: begin
                    pair01 <= {1'b0,lane_count[0]} + {1'b0,lane_count[1]};
                    pair23 <= {1'b0,lane_count[2]} + {1'b0,lane_count[3]};
                    state <= BUILD_BIN;
                end
                BUILD_BIN: begin
                    bin_total_q <= {1'b0,pair01} + {1'b0,pair23};
                    state <= BUILD_ACCUM;
                end
                BUILD_ACCUM: begin
                    accumulated <= {2'b0,cdf} + bin_total_q;
                    state <= BUILD_CDF;
                end
                BUILD_CDF: begin
                    cdf <= accumulated[COUNT_BITS-1:0];
                    if(accumulated > PIXELS) begin
                        state <= ERROR; protocol_error <= 1; m_valid <= 0;
                    end else if(accumulated <= {2'b0,cdf_min}) begin
                        state <= ADVANCE;
                    end else begin
                        state <= BUILD_DELTA;
                    end
                end
                BUILD_DELTA: begin
                    delta_q <= cdf - cdf_min;
                    state <= BUILD_SCALE;
                end
                BUILD_SCALE: begin
                    scaled_q <= (delta_extended << 8) - delta_extended;
                    state <= BUILD_ROUND;
                end
                BUILD_ROUND: begin
                    dividend <= scaled_q + (denominator_extended >> 1);
                    quotient <= 0; remainder <= 0; bits_left <= NUM_BITS;
                    state <= DIVIDE;
                end
                DIVIDE: begin
                    dividend <= {dividend[NUM_BITS-2:0],1'b0};
                    quotient <= quotient_next;
                    remainder <= subtract_divisor ? shifted_remainder-{1'b0,denominator} : shifted_remainder;
                    bits_left <= bits_left-1;
                    if(bits_left == 1) state <= ADVANCE;
                end
                ADVANCE: begin
                    if(bin_index == 255) begin state <= REMAP; rx <= 0; ry <= 0; end
                    else begin bin_index <= bin_index + 1'b1; state <= BUILD_READ; end
                end
                REMAP: if(replay_accept) begin
                    if(replay_last) state <= DRAIN_OUTPUT;
                    else if(rx == WORDS-1) begin rx <= 0; ry <= ry+1; end
                    else rx <= rx+1;
                end
                DRAIN_OUTPUT: if(m_valid && m_ready) begin state <= CLEAR; bin_index <= 0; end
                default: begin state <= ERROR; protocol_error <= 1; m_valid <= 0; end
            endcase
        end
    end
endmodule
