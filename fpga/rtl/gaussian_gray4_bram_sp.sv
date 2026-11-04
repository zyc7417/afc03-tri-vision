// Standalone grayscale 3x3 Gaussian. See README.txt for the complete contract.
// Two packed row memories (not a frame store), synchronous read-first accesses.
`timescale 1ns/1ps
module gaussian_gray4 #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080
) (
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready,
    input wire [127:0] s_data, input wire [3:0] s_keep,
    input wire s_sof, input wire s_eol, input wire s_eof,
    input wire s_bypass,
    output reg m_valid, input wire m_ready,
    output reg [127:0] m_data, output reg [3:0] m_keep,
    output reg m_sof, output reg m_eol, output reg m_eof,
    output reg protocol_error
);
    localparam integer WORDS = (WIDTH + 3) / 4;
    localparam integer ROW_AW = WORDS > 1 ? $clog2(WORDS) : 1;
    localparam integer ROW_DEPTH = (1 << ROW_AW);
    localparam integer TAIL = WIDTH - (WORDS - 1) * 4;
    localparam [3:0] TAIL_KEEP = (1 << TAIL) - 1;
    localparam [1:0] INPUT_ROWS = 0, BOTTOM_ROW = 1, DRAIN = 2;
    reg [1:0] state;
    reg frame_bypass;
    integer in_x, in_y, bottom_x;

    // Each bank represents one row, four 8-bit pixels per address. No reset:
    // validity/counters prevent reading an unwritten row into an output.
    // Padded power-of-two address space avoids a synthesized out-of-bounds
    // read guard on q0/q1, which TD cannot absorb into a BRAM read register.
    // Legal accesses still use only 0..WORDS-1; no frame data is padded/emitted.
    (* ram_style = "bram" *) reg [31:0] row0 [0:ROW_DEPTH-1];
    (* ram_style = "bram" *) reg [31:0] row1 [0:ROW_DEPTH-1];
    reg [31:0] q0, q1;
    reg [31:0] packed_input;
    wire step = !m_valid || m_ready;
    assign s_ready = rst_n && !protocol_error && state == INPUT_ROWS && step;
    wire accept = s_valid && s_ready;
    wire first_input = in_x == 0 && in_y == 0;
    wire last_col = in_x == WORDS - 1;
    wire last_input = last_col && in_y == HEIGHT - 1;
    wire bad_protocol = accept &&
        ((s_keep != (last_col ? TAIL_KEEP : 4'hf)) ||
         (s_sof != first_input) || (s_eol != last_col) ||
         (s_eof != last_input));
    wire real_issue = accept && !bad_protocol;
    wire bottom_issue = rst_n && !protocol_error && step && state == BOTTOM_ROW;
    wire issue = real_issue || bottom_issue;
    wire [31:0] issue_x = bottom_issue ? bottom_x : in_x;
    wire [ROW_AW-1:0] row_raddr = issue_x[ROW_AW-1:0];

    // Stage R: memory read and aligned input/mode/position registers.
    reg r_valid, r_bottom, r_mode;
    reg [31:0] r_input;
    integer r_x, r_y;
    // Stage V: 10-bit unrounded vertical sums and the middle row for bypass.
    reg v_valid, v_mode;
    reg [39:0] v_sum;
    reg [31:0] v_middle;
    integer v_x, v_y;
    // Pending V word supplies the horizontal right-neighbour look-ahead.
    reg p_valid, p_mode;
    reg [39:0] p_sum;
    reg [31:0] p_middle;
    reg [9:0] p_left, previous_last;
    integer p_x, p_y;

    reg [31:0] middle_word, top_word, bottom_word;
    reg [39:0] vertical_sum;
    reg [127:0] horizontal_data;
    reg [9:0] left_value, center_value, right_value;
    reg [11:0] rounded_value;
    reg [7:0] out_y;
    integer i, nvalid;
    always @* begin
        packed_input = 0;
        for (integer k = 0; k < 4; k = k + 1)
            if (s_keep[k]) packed_input[8*k +: 8] = s_data[32*k +: 8];

        // In row y, the write bank previously contained y-2; read-first
        // memory semantics preserve it in q0/q1 for this computation.
        middle_word = 0;
        top_word = 0;
        bottom_word = r_input;
        if (r_bottom) begin
            middle_word = ((HEIGHT - 1) % 2 == 0) ? q0 : q1;
            top_word = (HEIGHT == 1) ? middle_word :
                       (((HEIGHT - 1) % 2 == 0) ? q1 : q0);
            bottom_word = middle_word;
        end else if (r_y > 0) begin
            middle_word = (r_y % 2 == 0) ? q1 : q0;
            top_word = (r_y == 1) ? middle_word :
                       ((r_y % 2 == 0) ? q0 : q1);
        end
        if (!r_bottom && r_y == HEIGHT/2) bottom_word = middle_word;
        if (!r_bottom && r_y == HEIGHT/2+1) top_word = middle_word;
        vertical_sum = 0;
        for (integer k = 0; k < 4; k = k + 1)
            vertical_sum[10*k +: 10] =
                {2'b0,top_word[8*k +: 8]} +
                ({2'b0,middle_word[8*k +: 8]} << 1) +
                {2'b0,bottom_word[8*k +: 8]};

        horizontal_data = 0;
        left_value = 0; center_value = 0; right_value = 0;
        rounded_value = 0; out_y = 0;
        nvalid = (p_x == WORDS - 1) ? TAIL : 4;
        for (integer k = 0; k < 4; k = k + 1) begin
            if (k < nvalid) begin
                center_value = p_sum[10*k +: 10];
                left_value = (k == 0) ? p_left : p_sum[10*(k-1) +: 10];
                if (p_x == WORDS - 1 && k == nvalid - 1)
                    right_value = center_value;
                else if (k == 3)
                    right_value = v_sum[0 +: 10];
                else
                    right_value = p_sum[10*(k+1) +: 10];
                if (p_x*4+k == WIDTH/2) left_value = center_value;
                if (p_x*4+k == WIDTH/2-1) right_value = center_value;
                rounded_value = {2'b0,left_value} +
                                ({2'b0,center_value} << 1) +
                                {2'b0,right_value} + 12'd8;
                out_y = p_mode ? p_middle[8*k +: 8] : rounded_value[11:4];
                horizontal_data[32*k +: 32] = {8'd0,out_y,out_y,out_y};
            end
        end
    end

    // This block deliberately has no asynchronous reset on the memories or
    // read registers. Both banks perform at most one read and one same-address
    // write per enabled clock, in read-first style.
    always @(posedge clk) begin
        if (rst_n && issue) begin
            q0 <= row0[row_raddr];
            q1 <= row1[row_raddr];
            if (real_issue) begin
                if (in_y % 2 == 0) row0[row_raddr] <= packed_input;
                else row1[row_raddr] <= packed_input;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= INPUT_ROWS;
            in_x <= 0; in_y <= 0; bottom_x <= 0; frame_bypass <= 0;
            protocol_error <= 0;
            r_valid <= 0; r_bottom <= 0; r_mode <= 0;
            r_input <= 0; r_x <= 0; r_y <= 0;
            v_valid <= 0; v_mode <= 0; v_sum <= 0; v_middle <= 0;
            v_x <= 0; v_y <= 0;
            p_valid <= 0; p_mode <= 0; p_sum <= 0; p_middle <= 0;
            p_left <= 0; previous_last <= 0; p_x <= 0; p_y <= 0;
            m_valid <= 0; m_data <= 0; m_keep <= 0;
            m_sof <= 0; m_eol <= 0; m_eof <= 0;
        end else if (bad_protocol) begin
            // An invalid frame is not repaired silently. Previously delivered
            // beats cannot be recalled; discard the frame and reset externally.
            protocol_error <= 1;
            r_valid <= 0; v_valid <= 0; p_valid <= 0; m_valid <= 0;
        end else if (step && !protocol_error) begin
            m_valid <= p_valid && (p_x == WORDS - 1 || v_valid);
            if (p_valid && (p_x == WORDS - 1 || v_valid)) begin
                m_data <= horizontal_data;
                m_keep <= (p_x == WORDS - 1) ? TAIL_KEEP : 4'hf;
                m_sof <= p_x == 0 && p_y == 0;
                m_eol <= p_x == WORDS - 1;
                m_eof <= p_x == WORDS - 1 && p_y == HEIGHT - 1;
            end
            if (v_valid) begin
                p_valid <= 1;
                p_sum <= v_sum; p_middle <= v_middle; p_mode <= v_mode;
                p_x <= v_x; p_y <= v_y;
                p_left <= (v_x == 0) ? v_sum[0 +: 10] : previous_last;
                previous_last <= (v_x == WORDS - 1) ?
                                 v_sum[10*(TAIL-1) +: 10] : v_sum[30 +: 10];
            end else if (p_valid && p_x == WORDS - 1) begin
                p_valid <= 0;
            end

            v_valid <= r_valid && (r_bottom || r_y > 0);
            if (r_valid && (r_bottom || r_y > 0)) begin
                v_sum <= vertical_sum; v_middle <= middle_word; v_mode <= r_mode;
                v_x <= r_x; v_y <= r_bottom ? HEIGHT - 1 : r_y - 1;
            end
            r_valid <= issue;
            if (issue) begin
                r_bottom <= bottom_issue;
                r_input <= packed_input;
                r_x <= issue_x; r_y <= in_y;
                r_mode <= real_issue && first_input ? s_bypass : frame_bypass;
            end
            if (real_issue) begin
                if (first_input) frame_bypass <= s_bypass;
                if (last_input) begin
                    state <= BOTTOM_ROW; bottom_x <= 0;
                end else if (last_col) begin
                    in_x <= 0; in_y <= in_y + 1;
                end else in_x <= in_x + 1;
            end
            if (bottom_issue) begin
                if (bottom_x == WORDS - 1) state <= DRAIN;
                else bottom_x <= bottom_x + 1;
            end
            if (state == DRAIN && !r_valid && !v_valid && !p_valid && !m_valid) begin
                state <= INPUT_ROWS; in_x <= 0; in_y <= 0;
            end
        end
    end
endmodule
