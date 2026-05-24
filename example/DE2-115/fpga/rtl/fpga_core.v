/*

Copyright (c) 2020 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * FPGA core logic
 */
module fpga_core #
(
    parameter TARGET = "GENERIC"
)
(
    /*
     * Clock: 125MHz
     * Synchronous reset
     */
    input  wire       clk,
    input  wire       clk90,
    input  wire       rst,

    /*
     * GPIO
     */
    input  wire [3:0]  btn,
    input  wire [17:0] sw,
    output wire [8:0]  ledg,
    output wire [17:0] ledr,
    output wire [6:0]  hex0,
    output wire [6:0]  hex1,
    output wire [6:0]  hex2,
    output wire [6:0]  hex3,
    output wire [6:0]  hex4,
    output wire [6:0]  hex5,
    output wire [6:0]  hex6,
    output wire [6:0]  hex7,
    output wire [35:0] gpio,

    /*
     * Ethernet: 1000BASE-T PHY interface
     */
    input  wire       phy0_rx_clk,
    input  wire [3:0] phy0_rxd,
    input  wire       phy0_rx_ctl,
    output wire       phy0_tx_clk,
    output wire [3:0] phy0_txd,
    output wire       phy0_tx_ctl,
    output wire       phy0_reset_n,
    input  wire       phy0_int_n,

    input  wire       phy1_rx_clk,
    input  wire [3:0] phy1_rxd,
    input  wire       phy1_rx_ctl,
    output wire       phy1_tx_clk,
    output wire [3:0] phy1_txd,
    output wire       phy1_tx_ctl,
    output wire       phy1_reset_n,
    input  wire       phy1_int_n
);

// MAC 0 AXI stream interfaces
wire [7:0] rx0_axis_tdata;
wire       rx0_axis_tvalid;
wire       rx0_axis_tready;
wire       rx0_axis_tlast;
wire       rx0_axis_tuser;

wire [7:0] tx0_axis_tdata;
wire       tx0_axis_tvalid;
wire       tx0_axis_tready;
wire       tx0_axis_tlast;
wire       tx0_axis_tuser;

// MAC 1 AXI stream interfaces
wire [7:0] rx1_axis_tdata;
wire       rx1_axis_tvalid;
wire       rx1_axis_tready;
wire       rx1_axis_tlast;
wire       rx1_axis_tuser;

wire [7:0] tx1_axis_tdata;
wire       tx1_axis_tvalid;
wire       tx1_axis_tready;
wire       tx1_axis_tlast;
wire       tx1_axis_tuser;

wire forward_0_to_1 = rx0_axis_tvalid && rx0_axis_tready && rx0_axis_tlast && !rx0_axis_tuser;
wire forward_1_to_0 = rx1_axis_tvalid && rx1_axis_tready && rx1_axis_tlast && !rx1_axis_tuser;

// Transparent two-port bridge.  Frames are not parsed, filtered, or rewritten.
assign tx1_axis_tdata = rx0_axis_tdata;
assign tx1_axis_tvalid = rx0_axis_tvalid;
assign rx0_axis_tready = tx1_axis_tready;
assign tx1_axis_tlast = rx0_axis_tlast;
assign tx1_axis_tuser = rx0_axis_tuser;

assign tx0_axis_tdata = rx1_axis_tdata;
assign tx0_axis_tvalid = rx1_axis_tvalid;
assign rx1_axis_tready = tx0_axis_tready;
assign tx0_axis_tlast = rx1_axis_tlast;
assign tx0_axis_tuser = rx1_axis_tuser;

assign phy0_reset_n = ~rst;
assign phy1_reset_n = ~rst;

assign gpio = 36'd0;
assign ledr = sw;

reg [25:0] heartbeat_counter_reg = 26'd0;
reg [31:0] forward_count_0_to_1_reg = 32'd0;
reg [31:0] forward_count_1_to_0_reg = 32'd0;

always @(posedge clk) begin
    if (rst) begin
        heartbeat_counter_reg <= 26'd0;
        forward_count_0_to_1_reg <= 32'd0;
        forward_count_1_to_0_reg <= 32'd0;
    end else begin
        heartbeat_counter_reg <= heartbeat_counter_reg + 1'b1;

        if (forward_0_to_1) begin
            forward_count_0_to_1_reg <= forward_count_0_to_1_reg + 1'b1;
        end

        if (forward_1_to_0) begin
            forward_count_1_to_0_reg <= forward_count_1_to_0_reg + 1'b1;
        end
    end
end

assign ledg = {
    heartbeat_counter_reg[25],
    !phy1_int_n,
    !phy0_int_n,
    tx1_axis_tready,
    tx0_axis_tready,
    rx1_axis_tvalid,
    rx0_axis_tvalid,
    forward_count_1_to_0_reg[0],
    forward_count_0_to_1_reg[0]
};

assign hex0 = 7'b1111111;
assign hex1 = 7'b1111111;
assign hex2 = 7'b1111111;
assign hex3 = 7'b1111111;
assign hex4 = 7'b1111111;
assign hex5 = 7'b1111111;
assign hex6 = 7'b1111111;
assign hex7 = 7'b1111111;

eth_mac_1g_rgmii_fifo #(
    .TARGET(TARGET),
    .USE_CLK90("TRUE"),
    .ENABLE_PADDING(1),
    .MIN_FRAME_LENGTH(64),
    .TX_FIFO_DEPTH(4096),
    .TX_FRAME_FIFO(1),
    .RX_FIFO_DEPTH(4096),
    .RX_FRAME_FIFO(1)
)
eth_mac_inst (
    .gtx_clk(clk),
    .gtx_clk90(clk90),
    .gtx_rst(rst),
    .logic_clk(clk),
    .logic_rst(rst),

    .tx_axis_tdata(tx0_axis_tdata),
    .tx_axis_tkeep(1'b1),
    .tx_axis_tvalid(tx0_axis_tvalid),
    .tx_axis_tready(tx0_axis_tready),
    .tx_axis_tlast(tx0_axis_tlast),
    .tx_axis_tuser(tx0_axis_tuser),

    .rx_axis_tdata(rx0_axis_tdata),
    .rx_axis_tkeep(),
    .rx_axis_tvalid(rx0_axis_tvalid),
    .rx_axis_tready(rx0_axis_tready),
    .rx_axis_tlast(rx0_axis_tlast),
    .rx_axis_tuser(rx0_axis_tuser),

    .rgmii_rx_clk(phy0_rx_clk),
    .rgmii_rxd(phy0_rxd),
    .rgmii_rx_ctl(phy0_rx_ctl),
    .rgmii_tx_clk(phy0_tx_clk),
    .rgmii_txd(phy0_txd),
    .rgmii_tx_ctl(phy0_tx_ctl),

    .tx_error_underflow(),
    .tx_fifo_overflow(),
    .tx_fifo_bad_frame(),
    .tx_fifo_good_frame(),
    .rx_error_bad_frame(),
    .rx_error_bad_fcs(),
    .rx_fifo_overflow(),
    .rx_fifo_bad_frame(),
    .rx_fifo_good_frame(),
    .speed(),

    .cfg_ifg(8'd12),
    .cfg_tx_enable(1'b1),
    .cfg_rx_enable(1'b1)
);

eth_mac_1g_rgmii_fifo #(
    .TARGET(TARGET),
    .USE_CLK90("TRUE"),
    .ENABLE_PADDING(1),
    .MIN_FRAME_LENGTH(64),
    .TX_FIFO_DEPTH(4096),
    .TX_FRAME_FIFO(1),
    .RX_FIFO_DEPTH(4096),
    .RX_FRAME_FIFO(1)
)
eth_mac_1_inst (
    .gtx_clk(clk),
    .gtx_clk90(clk90),
    .gtx_rst(rst),
    .logic_clk(clk),
    .logic_rst(rst),

    .tx_axis_tdata(tx1_axis_tdata),
    .tx_axis_tkeep(1'b1),
    .tx_axis_tvalid(tx1_axis_tvalid),
    .tx_axis_tready(tx1_axis_tready),
    .tx_axis_tlast(tx1_axis_tlast),
    .tx_axis_tuser(tx1_axis_tuser),

    .rx_axis_tdata(rx1_axis_tdata),
    .rx_axis_tkeep(),
    .rx_axis_tvalid(rx1_axis_tvalid),
    .rx_axis_tready(rx1_axis_tready),
    .rx_axis_tlast(rx1_axis_tlast),
    .rx_axis_tuser(rx1_axis_tuser),

    .rgmii_rx_clk(phy1_rx_clk),
    .rgmii_rxd(phy1_rxd),
    .rgmii_rx_ctl(phy1_rx_ctl),
    .rgmii_tx_clk(phy1_tx_clk),
    .rgmii_txd(phy1_txd),
    .rgmii_tx_ctl(phy1_tx_ctl),

    .tx_error_underflow(),
    .tx_fifo_overflow(),
    .tx_fifo_bad_frame(),
    .tx_fifo_good_frame(),
    .rx_error_bad_frame(),
    .rx_error_bad_fcs(),
    .rx_fifo_overflow(),
    .rx_fifo_bad_frame(),
    .rx_fifo_good_frame(),
    .speed(),

    .cfg_ifg(8'd12),
    .cfg_tx_enable(1'b1),
    .cfg_rx_enable(1'b1)
);

endmodule

`resetall
