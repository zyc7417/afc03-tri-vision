// VID007 cropped RAW10x4: p0 is data[9:0], then p1,p2,p3.
// Exactly one GBRG 2x2 cell produces one GRAY8 pixel; this is NOT
// full-resolution demosaic. No sensor backpressure. Outputs are provisional
// until commit_valid; a sink must discard the entire frame on abort_valid.
// Any malformed input or overflow locks this instance until reset.
`timescale 1ns/1ps
module quad_board_raw10_gbrg_half_gray #(
    parameter integer WIDTH=1920, HEIGHT=1080, FIFO_DEPTH=8
)(
    input wire clk, input wire rst_n,
    input wire s_valid, input wire [39:0] s_data,
    input wire s_sof, input wire s_eol, input wire [31:0] s_frame_id,
    input wire s_frame_done, input wire s_frame_good, input wire s_error,
    output wire m_valid, input wire m_ready, output wire [15:0] m_data,
    output wire m_sof, output wire m_eol, output wire m_eof,
    output wire [31:0] m_frame_id,
    output reg commit_valid, output reg abort_valid,
    output reg [31:0] verdict_frame_id,
    output reg fault, output reg [7:0] fault_code,
    output reg [31:0] input_beats, output reg [31:0] output_pixels
);
    localparam integer COLS=WIDTH/4;
    localparam integer AW=(COLS>1)?$clog2(COLS):1;
    localparam integer PW=(FIFO_DEPTH>1)?$clog2(FIFO_DEPTH):1;
    // Synchronous row store: 40 x 480 = 19200 useful bits at full HD.
    reg [39:0] previous_row [0:COLS-1];
    reg [39:0] top_q, bottom_q;
    reg pipe_valid, pipe_sof, pipe_eol, pipe_eof;
    reg product_valid,sum_valid,result_valid;
    reg [2:0]product_flags,sum_flags,result_flags;
    reg [16:0]p_r[0:1],p_g0[0:1],p_g1[0:1],p_b[0:1];
    reg [17:0]sum_rg[0:1],sum_gb[0:1];
    reg [18:0]rounded[0:1];
    wire pipeline_busy=pipe_valid||product_valid||sum_valid||result_valid;
    integer lane;
    reg [18:0] fifo [0:FIFO_DEPTH-1];
    reg [PW-1:0] wr_ptr, rd_ptr;
    reg [PW:0] count;
    reg [AW-1:0] col;
    reg [15:0] row;
    reg active, pixels_complete, guard_passed;
    reg [31:0] frame_id;
    wire pop=m_valid&&m_ready;
    wire space=(count<FIFO_DEPTH)||pop;
    wire push=result_valid&&!fault&&space;
    wire start_ok=s_valid&&s_sof&&!active&&(count==0)&&!pipeline_busy;
    wire [15:0] this_row=start_ok?16'd0:row;
    wire [AW-1:0] this_col=start_ok?{AW{1'b0}}:col;
    wire last_col=(this_col==COLS-1);
    wire last_input=(this_row==HEIGHT-1)&&last_col;
    wire input_ok=s_valid&&!fault&&(start_ok||active)&&!pixels_complete&&
                  !(s_sof&&!start_ok)&&(s_eol==last_col);
    wire [7:0] errors=
        ((s_error)?8'h01:8'h00)|
        ((s_valid&&(!(start_ok||active)||(s_sof&&!start_ok)||pixels_complete))?8'h02:8'h00)|
        ((s_valid&&(start_ok||active)&&(s_eol!=last_col))?8'h04:8'h00)|
        ((result_valid&&!space)?8'h08:8'h00)|
        ((s_frame_done&&(!active||!(pixels_complete||(input_ok&&last_input))||!s_frame_good))?8'h10:8'h00)|
        ((active&&(s_valid||s_frame_done)&&(s_frame_id!=frame_id))?8'h20:8'h00);
    wire [7:0]gray0 = rounded[0] >= 19'd262144 ? 8'd255 : rounded[0][17:10];
    wire [7:0]gray1 = rounded[1] >= 19'd262144 ? 8'd255 : rounded[1][17:10];
    wire [15:0]gray_pair={gray1,gray0};
    assign m_valid=!fault&&(count!=0);
    assign {m_eof,m_eol,m_sof,m_data}=fifo[rd_ptr];
    assign m_frame_id=frame_id;
    initial begin
        if(WIDTH<4||WIDTH%4||HEIGHT<2||HEIGHT%2||HEIGHT>65535||FIFO_DEPTH<2||
           (FIFO_DEPTH&(FIFO_DEPTH-1))) $fatal(1,"BAD_GEOMETRY_OR_FIFO");
    end
    always @(posedge clk) begin
        if(input_ok&&errors==0) begin
            if(!this_row[0]) previous_row[this_col]<=s_data;
            else begin top_q<=previous_row[this_col]; bottom_q<=s_data; end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pipe_valid<=0;pipe_sof<=0;pipe_eol<=0;pipe_eof<=0;
            product_valid<=0;sum_valid<=0;result_valid<=0;
            product_flags<=0;sum_flags<=0;result_flags<=0;
            for(lane=0;lane<2;lane=lane+1)begin
                p_r[lane]<=0;p_g0[lane]<=0;p_g1[lane]<=0;p_b[lane]<=0;
                sum_rg[lane]<=0;sum_gb[lane]<=0;rounded[lane]<=0;
            end
            wr_ptr<=0;rd_ptr<=0;count<=0;col<=0;row<=0;active<=0;
            pixels_complete<=0;guard_passed<=0;frame_id<=0;
            commit_valid<=0;abort_valid<=0;verdict_frame_id<=0;
            fault<=0;fault_code<=0;input_beats<=0;output_pixels<=0;
        end else begin
            commit_valid<=0;abort_valid<=0;pipe_valid<=0;product_valid<=0;sum_valid<=0;result_valid<=0;
            if(!fault) begin
                if(errors!=0) begin
                    fault<=1;fault_code<=errors;abort_valid<=1;
                    verdict_frame_id<=start_ok?s_frame_id:frame_id;
                    active<=0;count<=0;wr_ptr<=0;rd_ptr<=0;
                end else begin
                    product_valid<=pipe_valid;sum_valid<=product_valid;result_valid<=sum_valid;
                    if(pipe_valid)begin
                        product_flags<={pipe_eof,pipe_eol,pipe_sof};
                        for(lane=0;lane<2;lane=lane+1)begin
                            p_r[lane]<=bottom_q[lane*20+:10]*17'd77;
                            p_g0[lane]<=top_q[lane*20+:10]*17'd75;
                            p_g1[lane]<=bottom_q[lane*20+10+:10]*17'd75;
                            p_b[lane]<=top_q[lane*20+10+:10]*17'd29;
                        end
                    end
                    if(product_valid)begin
                        sum_flags<=product_flags;
                        for(lane=0;lane<2;lane=lane+1)begin
                            sum_rg[lane]<={1'b0,p_r[lane]}+{1'b0,p_g0[lane]};
                            sum_gb[lane]<={1'b0,p_g1[lane]}+{1'b0,p_b[lane]};
                        end
                    end
                    if(sum_valid)begin
                        result_flags<=sum_flags;
                        for(lane=0;lane<2;lane=lane+1)rounded[lane]<={1'b0,sum_rg[lane]}+{1'b0,sum_gb[lane]}+19'd512;
                    end
                    if(push) begin
                        fifo[wr_ptr]<={result_flags,gray_pair};
                        wr_ptr<=wr_ptr+1'b1;
                    end
                    if(pop) begin rd_ptr<=rd_ptr+1'b1;output_pixels<=output_pixels+2;end
                    case({push,pop})
                        2'b10:count<=count+1'b1;
                        2'b01:count<=count-1'b1;
                        default:count<=count;
                    endcase
                    if(input_ok) begin
                        if(start_ok) begin
                            active<=1;frame_id<=s_frame_id;guard_passed<=0;
                            pixels_complete<=0;input_beats<=1;output_pixels<=0;
                        end else input_beats<=input_beats+1;
                        if(this_row[0]) begin
                            pipe_valid<=1;pipe_sof<=(this_row==1&&this_col==0);
                            pipe_eol<=last_col;pipe_eof<=last_input;
                        end
                        if(last_col) begin col<=0;row<=this_row+1'b1;end
                        else begin col<=this_col+1'b1;row<=this_row;end
                        if(last_input)pixels_complete<=1;
                    end
                    if(s_frame_done)guard_passed<=1;
                    if(active&&pixels_complete&&guard_passed&&!pipeline_busy&&count==0)begin
                        commit_valid<=1;verdict_frame_id<=frame_id;active<=0;
                        pixels_complete<=0;guard_passed<=0;
                    end
                end
            end
        end
    end
endmodule
