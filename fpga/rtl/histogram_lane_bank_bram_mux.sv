// Independent mapping candidate: exactly one explicit write port and read port.
// Algorithm, single-cycle read latency, pending commit and forwarding unchanged.
`timescale 1ns/1ps
module histogram_lane_bank #(parameter integer COUNT_BITS = 21) (
    input wire clk, input wire rst_n, input wire abort,
    input wire clear_enable, input wire [7:0] clear_address,
    input wire sample_valid, input wire [7:0] sample_value,
    input wire scan_enable, input wire [7:0] scan_address,
    output reg [COUNT_BITS-1:0] scan_data,
    output reg pending
);
    (* ram_style = "bram" *) reg [COUNT_BITS-1:0] counts [0:255];
    reg [7:0] pending_address, last_address;
    reg last_valid;
    reg [COUNT_BITS-1:0] last_value;
    wire commit = rst_n && pending && !abort && !clear_enable;
    wire [COUNT_BITS-1:0] incremented =
        ((last_valid && last_address == pending_address) ? last_value : scan_data) + 1'b1;
    wire write_enable = rst_n && (clear_enable || commit);
    wire [7:0] write_address = clear_enable ? clear_address : pending_address;
    wire [COUNT_BITS-1:0] write_data = clear_enable ? {COUNT_BITS{1'b0}} : incremented;
    wire read_enable = rst_n && !abort && !clear_enable && (sample_valid || scan_enable);
    wire [7:0] read_address = sample_valid ? sample_value : scan_address;
    // Synchronous simple-dual-port RAM template. Address selection is outside RAM.
    always @(posedge clk) begin
        if(write_enable) counts[write_address] <= write_data;
        if(read_enable) scan_data <= counts[read_address];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pending <= 0; pending_address <= 0;
            last_valid <= 0; last_address <= 0; last_value <= 0;
        end else if(abort || clear_enable) begin
            pending <= 0; last_valid <= 0;
        end else begin
            pending <= sample_valid;
            if(sample_valid) pending_address <= sample_value;
            if(commit) begin
                last_valid <= 1; last_address <= pending_address; last_value <= incremented;
            end
        end
    end
endmodule
