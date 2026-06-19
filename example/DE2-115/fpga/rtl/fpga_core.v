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
    output wire       phy0_mdc,
    input  wire       phy0_mdio_i,
    output wire       phy0_mdio_o,
    output wire       phy0_mdio_t,
    input  wire       phy0_int_n,

    input  wire       phy1_rx_clk,
    input  wire [3:0] phy1_rxd,
    input  wire       phy1_rx_ctl,
    output wire       phy1_tx_clk,
    output wire [3:0] phy1_txd,
    output wire       phy1_tx_ctl,
    output wire       phy1_reset_n,
    output wire       phy1_mdc,
    input  wire       phy1_mdio_i,
    output wire       phy1_mdio_o,
    output wire       phy1_mdio_t,
    input  wire       phy1_int_n
);

localparam [6:0] HEX_OFF = 7'b1111111;

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

wire [7:0]  bridge_0_to_1_axis_tdata;
wire        bridge_0_to_1_axis_tvalid;
wire        bridge_0_to_1_axis_tready;
wire        bridge_0_to_1_axis_tlast;
wire        bridge_0_to_1_axis_tuser;
wire        bridge_0_to_1_error_header_early_termination;

wire [7:0]  processed_0_to_1_axis_tdata;
wire        processed_0_to_1_axis_tvalid;
wire        processed_0_to_1_axis_tready;
wire        processed_0_to_1_axis_tlast;
wire        processed_0_to_1_axis_tuser;

wire [7:0]  bridge_1_to_0_axis_tdata;
wire        bridge_1_to_0_axis_tvalid;
wire        bridge_1_to_0_axis_tready;
wire        bridge_1_to_0_axis_tlast;
wire        bridge_1_to_0_axis_tuser;
wire        bridge_1_to_0_error_header_early_termination;

wire [7:0]  processed_1_to_0_axis_tdata;
wire        processed_1_to_0_axis_tvalid;
wire        processed_1_to_0_axis_tready;
wire        processed_1_to_0_axis_tlast;
wire        processed_1_to_0_axis_tuser;

wire [15:0] packet_load_status_0_to_1;
wire [15:0] packet_load_status_1_to_0;

wire fifo_0_to_1_overflow;
wire fifo_0_to_1_bad_frame;
wire fifo_0_to_1_good_frame;
wire fifo_1_to_0_overflow;
wire fifo_1_to_0_bad_frame;
wire fifo_1_to_0_good_frame;

wire mac0_rx_error_bad_frame;
wire mac0_rx_error_bad_fcs;
wire mac0_rx_fifo_overflow;
wire mac0_rx_fifo_bad_frame;
wire mac0_rx_fifo_good_frame;
wire mac0_tx_fifo_overflow;
wire mac0_tx_fifo_bad_frame;
wire mac0_tx_fifo_good_frame;
wire [1:0] mac0_speed;

wire mac1_rx_error_bad_frame;
wire mac1_rx_error_bad_fcs;
wire mac1_rx_fifo_overflow;
wire mac1_rx_fifo_bad_frame;
wire mac1_rx_fifo_good_frame;
wire mac1_tx_fifo_overflow;
wire mac1_tx_fifo_bad_frame;
wire mac1_tx_fifo_good_frame;
wire [1:0] mac1_speed;

wire phy0_config_done;
wire phy1_config_done;
wire phy_config_done = phy0_config_done && phy1_config_done;
wire phy0_config_busy;
wire phy1_config_busy;
wire phy0_link_up;
wire phy1_link_up;
wire phy0_link_1g;
wire phy1_link_1g;
wire phy0_full_duplex;
wire phy1_full_duplex;
wire phy0_link_live = phy0_config_done && phy0_link_up && phy0_full_duplex;
wire phy1_link_live = phy1_config_done && phy1_link_up && phy1_full_duplex;
wire phy0_link_ready = phy0_link_live && phy0_link_1g;
wire phy1_link_ready = phy1_link_live && phy1_link_1g;
wire phy_link_ready = phy0_link_ready && phy1_link_ready;

wire bridge_0_to_1_frame = bridge_0_to_1_axis_tvalid && bridge_0_to_1_axis_tready && bridge_0_to_1_axis_tlast;
wire bridge_1_to_0_frame = bridge_1_to_0_axis_tvalid && bridge_1_to_0_axis_tready && bridge_1_to_0_axis_tlast;
wire forward_0_to_1 = bridge_0_to_1_frame && !bridge_0_to_1_axis_tuser;
wire forward_1_to_0 = bridge_1_to_0_frame && !bridge_1_to_0_axis_tuser;

wire rx0_frame = rx0_axis_tvalid && rx0_axis_tready && rx0_axis_tlast;
wire rx1_frame = rx1_axis_tvalid && rx1_axis_tready && rx1_axis_tlast;
wire tx0_frame = tx0_axis_tvalid && tx0_axis_tready && tx0_axis_tlast;
wire tx1_frame = tx1_axis_tvalid && tx1_axis_tready && tx1_axis_tlast;

reg [23:0] phy_reset_counter_reg = 24'd0;
reg phy_reset_n_reg = 1'b0;

wire mac_rst = rst || !phy_reset_n_reg || !phy_config_done;

always @(posedge clk) begin
    if (rst) begin
        phy_reset_counter_reg <= 24'd0;
        phy_reset_n_reg <= 1'b0;
    end else if (!phy_reset_n_reg) begin
        phy_reset_counter_reg <= phy_reset_counter_reg + 1'b1;

        if (&phy_reset_counter_reg) begin
            phy_reset_n_reg <= 1'b1;
        end
    end
end

assign phy0_reset_n = phy_reset_n_reg;
assign phy1_reset_n = phy_reset_n_reg;

assign gpio = {4'd0, packet_load_status_1_to_0, packet_load_status_0_to_1};

mdio_phy_config #(
    .PHY_ADDR(5'b10000)
)
phy0_mdio_config_inst (
    .clk(clk),
    .rst(rst),
    .start(phy_reset_n_reg),
    .mdc(phy0_mdc),
    .mdio_i(phy0_mdio_i),
    .mdio_o(phy0_mdio_o),
    .mdio_t(phy0_mdio_t),
    .busy(phy0_config_busy),
    .done(phy0_config_done),
    .link_up(phy0_link_up),
    .link_1g(phy0_link_1g),
    .full_duplex(phy0_full_duplex)
);

mdio_phy_config #(
    .PHY_ADDR(5'b10001)
)
phy1_mdio_config_inst (
    .clk(clk),
    .rst(rst),
    .start(phy_reset_n_reg),
    .mdc(phy1_mdc),
    .mdio_i(phy1_mdio_i),
    .mdio_o(phy1_mdio_o),
    .mdio_t(phy1_mdio_t),
    .busy(phy1_config_busy),
    .done(phy1_config_done),
    .link_up(phy1_link_up),
    .link_1g(phy1_link_1g),
    .full_duplex(phy1_full_duplex)
);

reg [25:0] heartbeat_counter_reg = 26'd0;
reg [31:0] forward_count_0_to_1_reg = 32'd0;
reg [31:0] forward_count_1_to_0_reg = 32'd0;
reg [23:0] rx0_activity_timer_reg = 24'd0;
reg [23:0] rx1_activity_timer_reg = 24'd0;
reg [23:0] tx0_activity_timer_reg = 24'd0;
reg [23:0] tx1_activity_timer_reg = 24'd0;

always @(posedge clk) begin
    if (rst) begin
        heartbeat_counter_reg <= 26'd0;
    end else begin
        heartbeat_counter_reg <= heartbeat_counter_reg + 1'b1;
    end

    if (mac_rst) begin
        forward_count_0_to_1_reg <= 32'd0;
        forward_count_1_to_0_reg <= 32'd0;
        rx0_activity_timer_reg <= 24'd0;
        rx1_activity_timer_reg <= 24'd0;
        tx0_activity_timer_reg <= 24'd0;
        tx1_activity_timer_reg <= 24'd0;
    end else begin
        if (rx0_activity_timer_reg != 0) begin
            rx0_activity_timer_reg <= rx0_activity_timer_reg - 1'b1;
        end

        if (rx1_activity_timer_reg != 0) begin
            rx1_activity_timer_reg <= rx1_activity_timer_reg - 1'b1;
        end

        if (tx0_activity_timer_reg != 0) begin
            tx0_activity_timer_reg <= tx0_activity_timer_reg - 1'b1;
        end

        if (tx1_activity_timer_reg != 0) begin
            tx1_activity_timer_reg <= tx1_activity_timer_reg - 1'b1;
        end

        if (rx0_frame) begin
            rx0_activity_timer_reg <= {24{1'b1}};
        end

        if (rx1_frame) begin
            rx1_activity_timer_reg <= {24{1'b1}};
        end

        if (tx0_frame) begin
            tx0_activity_timer_reg <= {24{1'b1}};
        end

        if (tx1_frame) begin
            tx1_activity_timer_reg <= {24{1'b1}};
        end

        if (forward_0_to_1) begin
            forward_count_0_to_1_reg <= forward_count_0_to_1_reg + 1'b1;
        end

        if (forward_1_to_0) begin
            forward_count_1_to_0_reg <= forward_count_1_to_0_reg + 1'b1;
        end
    end
end

assign ledg = {
    !mac_rst,
    phy1_link_live,
    phy0_link_live,
    mac0_rx_error_bad_frame || mac0_rx_error_bad_fcs || mac1_rx_error_bad_frame || mac1_rx_error_bad_fcs ||
        bridge_0_to_1_error_header_early_termination || bridge_1_to_0_error_header_early_termination,
    tx1_activity_timer_reg != 0,
    tx0_activity_timer_reg != 0,
    rx1_activity_timer_reg != 0,
    rx0_activity_timer_reg != 0,
    heartbeat_counter_reg[25]
};

assign ledr = {
    phy_link_ready,
    fifo_1_to_0_overflow,
    fifo_0_to_1_overflow,
    mac1_rx_fifo_overflow,
    mac0_rx_fifo_overflow,
    mac1_tx_fifo_overflow,
    mac0_tx_fifo_overflow,
    mac1_rx_fifo_bad_frame,
    mac0_rx_fifo_bad_frame,
    mac1_tx_fifo_bad_frame,
    mac0_tx_fifo_bad_frame,
    phy1_link_up,
    phy1_link_1g,
    phy0_link_up,
    phy0_link_1g,
    heartbeat_counter_reg[25],
    forward_count_1_to_0_reg[0],
    forward_count_0_to_1_reg[0]
};

assign hex0 = HEX_OFF;
assign hex1 = HEX_OFF;
assign hex2 = HEX_OFF;
assign hex3 = HEX_OFF;
assign hex4 = HEX_OFF;
assign hex5 = HEX_OFF;
assign hex6 = HEX_OFF;
assign hex7 = HEX_OFF;

// Transparent two-port bridge.  Each received frame is decoded into Ethernet
// header fields plus payload, then rebuilt unchanged before it enters the
// existing forwarding FIFOs.  The MACs still remove/check the receive FCS and
// generate a new transmit FCS.
de2_eth_frame_rebuild_8
frame_rebuild_0_to_1_inst (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(rx0_axis_tdata),
    .s_axis_tvalid(rx0_axis_tvalid),
    .s_axis_tready(rx0_axis_tready),
    .s_axis_tlast(rx0_axis_tlast),
    .s_axis_tuser(rx0_axis_tuser),

    .m_axis_tdata(bridge_0_to_1_axis_tdata),
    .m_axis_tvalid(bridge_0_to_1_axis_tvalid),
    .m_axis_tready(bridge_0_to_1_axis_tready),
    .m_axis_tlast(bridge_0_to_1_axis_tlast),
    .m_axis_tuser(bridge_0_to_1_axis_tuser),

    .error_header_early_termination(bridge_0_to_1_error_header_early_termination)
);

de2_eth_frame_rebuild_8
frame_rebuild_1_to_0_inst (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(rx1_axis_tdata),
    .s_axis_tvalid(rx1_axis_tvalid),
    .s_axis_tready(rx1_axis_tready),
    .s_axis_tlast(rx1_axis_tlast),
    .s_axis_tuser(rx1_axis_tuser),

    .m_axis_tdata(bridge_1_to_0_axis_tdata),
    .m_axis_tvalid(bridge_1_to_0_axis_tvalid),
    .m_axis_tready(bridge_1_to_0_axis_tready),
    .m_axis_tlast(bridge_1_to_0_axis_tlast),
    .m_axis_tuser(bridge_1_to_0_axis_tuser),

    .error_header_early_termination(bridge_1_to_0_error_header_early_termination)
);

de2_packet_processing_load_8 #(
    .STATE_WIDTH(10240),
    .STATUS_WIDTH(16)
)
packet_processing_load_0_to_1_inst (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(bridge_0_to_1_axis_tdata),
    .s_axis_tvalid(bridge_0_to_1_axis_tvalid),
    .s_axis_tready(bridge_0_to_1_axis_tready),
    .s_axis_tlast(bridge_0_to_1_axis_tlast),
    .s_axis_tuser(bridge_0_to_1_axis_tuser),

    .m_axis_tdata(processed_0_to_1_axis_tdata),
    .m_axis_tvalid(processed_0_to_1_axis_tvalid),
    .m_axis_tready(processed_0_to_1_axis_tready),
    .m_axis_tlast(processed_0_to_1_axis_tlast),
    .m_axis_tuser(processed_0_to_1_axis_tuser),

    .status(packet_load_status_0_to_1)
);

de2_packet_processing_load_8 #(
    .STATE_WIDTH(10240),
    .STATUS_WIDTH(16)
)
packet_processing_load_1_to_0_inst (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(bridge_1_to_0_axis_tdata),
    .s_axis_tvalid(bridge_1_to_0_axis_tvalid),
    .s_axis_tready(bridge_1_to_0_axis_tready),
    .s_axis_tlast(bridge_1_to_0_axis_tlast),
    .s_axis_tuser(bridge_1_to_0_axis_tuser),

    .m_axis_tdata(processed_1_to_0_axis_tdata),
    .m_axis_tvalid(processed_1_to_0_axis_tvalid),
    .m_axis_tready(processed_1_to_0_axis_tready),
    .m_axis_tlast(processed_1_to_0_axis_tlast),
    .m_axis_tuser(processed_1_to_0_axis_tuser),

    .status(packet_load_status_1_to_0)
);

axis_fifo #(
    .DEPTH(8192),
    .DATA_WIDTH(8),
    .KEEP_ENABLE(0),
    .ID_ENABLE(0),
    .DEST_ENABLE(0),
    .USER_ENABLE(1),
    .USER_WIDTH(1),
    .FRAME_FIFO(1),
    .DROP_BAD_FRAME(0),
    .DROP_WHEN_FULL(0)
)
bridge_fifo_0_to_1 (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(processed_0_to_1_axis_tdata),
    .s_axis_tkeep(1'b1),
    .s_axis_tvalid(processed_0_to_1_axis_tvalid),
    .s_axis_tready(processed_0_to_1_axis_tready),
    .s_axis_tlast(processed_0_to_1_axis_tlast),
    .s_axis_tid(8'd0),
    .s_axis_tdest(8'd0),
    .s_axis_tuser(processed_0_to_1_axis_tuser),

    .m_axis_tdata(tx1_axis_tdata),
    .m_axis_tkeep(),
    .m_axis_tvalid(tx1_axis_tvalid),
    .m_axis_tready(tx1_axis_tready),
    .m_axis_tlast(tx1_axis_tlast),
    .m_axis_tid(),
    .m_axis_tdest(),
    .m_axis_tuser(tx1_axis_tuser),

    .pause_req(1'b0),
    .pause_ack(),

    .status_depth(),
    .status_depth_commit(),
    .status_overflow(fifo_0_to_1_overflow),
    .status_bad_frame(fifo_0_to_1_bad_frame),
    .status_good_frame(fifo_0_to_1_good_frame)
);

axis_fifo #(
    .DEPTH(8192),
    .DATA_WIDTH(8),
    .KEEP_ENABLE(0),
    .ID_ENABLE(0),
    .DEST_ENABLE(0),
    .USER_ENABLE(1),
    .USER_WIDTH(1),
    .FRAME_FIFO(1),
    .DROP_BAD_FRAME(0),
    .DROP_WHEN_FULL(0)
)
bridge_fifo_1_to_0 (
    .clk(clk),
    .rst(mac_rst),

    .s_axis_tdata(processed_1_to_0_axis_tdata),
    .s_axis_tkeep(1'b1),
    .s_axis_tvalid(processed_1_to_0_axis_tvalid),
    .s_axis_tready(processed_1_to_0_axis_tready),
    .s_axis_tlast(processed_1_to_0_axis_tlast),
    .s_axis_tid(8'd0),
    .s_axis_tdest(8'd0),
    .s_axis_tuser(processed_1_to_0_axis_tuser),

    .m_axis_tdata(tx0_axis_tdata),
    .m_axis_tkeep(),
    .m_axis_tvalid(tx0_axis_tvalid),
    .m_axis_tready(tx0_axis_tready),
    .m_axis_tlast(tx0_axis_tlast),
    .m_axis_tid(),
    .m_axis_tdest(),
    .m_axis_tuser(tx0_axis_tuser),

    .pause_req(1'b0),
    .pause_ack(),

    .status_depth(),
    .status_depth_commit(),
    .status_overflow(fifo_1_to_0_overflow),
    .status_bad_frame(fifo_1_to_0_bad_frame),
    .status_good_frame(fifo_1_to_0_good_frame)
);

eth_mac_1g_rgmii_fifo #(
    .TARGET(TARGET),
    .USE_CLK90("FALSE"),
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
    .gtx_rst(mac_rst),
    .logic_clk(clk),
    .logic_rst(mac_rst),

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
    .tx_fifo_overflow(mac0_tx_fifo_overflow),
    .tx_fifo_bad_frame(mac0_tx_fifo_bad_frame),
    .tx_fifo_good_frame(mac0_tx_fifo_good_frame),
    .rx_error_bad_frame(mac0_rx_error_bad_frame),
    .rx_error_bad_fcs(mac0_rx_error_bad_fcs),
    .rx_fifo_overflow(mac0_rx_fifo_overflow),
    .rx_fifo_bad_frame(mac0_rx_fifo_bad_frame),
    .rx_fifo_good_frame(mac0_rx_fifo_good_frame),
    .speed(mac0_speed),

    .cfg_ifg(8'd12),
    .cfg_tx_enable(1'b1),
    .cfg_rx_enable(1'b1)
);

eth_mac_1g_rgmii_fifo #(
    .TARGET(TARGET),
    .USE_CLK90("FALSE"),
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
    .gtx_rst(mac_rst),
    .logic_clk(clk),
    .logic_rst(mac_rst),

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
    .tx_fifo_overflow(mac1_tx_fifo_overflow),
    .tx_fifo_bad_frame(mac1_tx_fifo_bad_frame),
    .tx_fifo_good_frame(mac1_tx_fifo_good_frame),
    .rx_error_bad_frame(mac1_rx_error_bad_frame),
    .rx_error_bad_fcs(mac1_rx_error_bad_fcs),
    .rx_fifo_overflow(mac1_rx_fifo_overflow),
    .rx_fifo_bad_frame(mac1_rx_fifo_bad_frame),
    .rx_fifo_good_frame(mac1_rx_fifo_good_frame),
    .speed(mac1_speed),

    .cfg_ifg(8'd12),
    .cfg_tx_enable(1'b1),
    .cfg_rx_enable(1'b1)
);

endmodule

module de2_packet_processing_load_8 #
(
    parameter STATE_WIDTH = 12288,
    parameter STATUS_WIDTH = 16
)
(
    input  wire                    clk,
    input  wire                    rst,

    input  wire [7:0]              s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output wire                    s_axis_tready,
    input  wire                    s_axis_tlast,
    input  wire                    s_axis_tuser,

    output wire [7:0]              m_axis_tdata,
    output wire                    m_axis_tvalid,
    input  wire                    m_axis_tready,
    output wire                    m_axis_tlast,
    output wire                    m_axis_tuser,

    output wire [STATUS_WIDTH-1:0] status
);

localparam STATUS_STRIDE = STATE_WIDTH / STATUS_WIDTH;
localparam MIX_SEGMENT_WIDTH = 2048;
localparam MIX_SEGMENT_COUNT = STATE_WIDTH / MIX_SEGMENT_WIDTH;
localparam [STATE_WIDTH-1:0] RESET_PATTERN = {STATE_WIDTH{1'b1}};

wire axis_beat = s_axis_tvalid && s_axis_tready;

(* preserve, noprune *) reg [STATE_WIDTH-1:0] packet_mix_reg = RESET_PATTERN;
(* preserve, noprune *) reg [31:0] byte_count_reg = 32'd0;
(* preserve, noprune *) reg [31:0] frame_count_reg = 32'd0;

wire [STATE_WIDTH-1:0] packet_mix_next;

assign s_axis_tready = m_axis_tready;

assign m_axis_tdata = s_axis_tdata;
assign m_axis_tvalid = s_axis_tvalid;
assign m_axis_tlast = s_axis_tlast;
assign m_axis_tuser = s_axis_tuser;

genvar status_index;
genvar mix_segment;
genvar mix_index;
generate
    for (mix_segment = 0; mix_segment < MIX_SEGMENT_COUNT; mix_segment = mix_segment + 1) begin : mix_segment_gen
        for (mix_index = 0; mix_index < MIX_SEGMENT_WIDTH; mix_index = mix_index + 1) begin : mix_gen
            localparam ABS_INDEX = mix_segment*MIX_SEGMENT_WIDTH + mix_index;
            localparam TAP_INDEX = (ABS_INDEX + 13) % STATE_WIDTH;
            localparam DATA_INDEX = ABS_INDEX % 8;
            localparam BYTE_COUNT_INDEX = ABS_INDEX % 32;
            localparam FRAME_COUNT_INDEX = (ABS_INDEX + 11) % 32;
            localparam LAST_ENABLE = (ABS_INDEX % 5) == 0;
            localparam USER_ENABLE = (ABS_INDEX % 7) == 0;

            assign packet_mix_next[ABS_INDEX] =
                packet_mix_reg[ABS_INDEX] ^
                packet_mix_reg[TAP_INDEX] ^
                s_axis_tdata[DATA_INDEX] ^
                byte_count_reg[BYTE_COUNT_INDEX] ^
                frame_count_reg[FRAME_COUNT_INDEX] ^
                (LAST_ENABLE ? s_axis_tlast : 1'b0) ^
                (USER_ENABLE ? s_axis_tuser : 1'b0);
        end
    end

    for (status_index = 0; status_index < STATUS_WIDTH; status_index = status_index + 1) begin : status_gen
        assign status[status_index] =
            packet_mix_reg[status_index*STATUS_STRIDE] ^
            packet_mix_reg[status_index*STATUS_STRIDE + STATUS_STRIDE/2] ^
            packet_mix_reg[status_index*STATUS_STRIDE + STATUS_STRIDE-1] ^
            byte_count_reg[status_index] ^
            frame_count_reg[status_index];
    end
endgenerate

always @(posedge clk) begin
    if (rst) begin
        packet_mix_reg <= RESET_PATTERN;
        byte_count_reg <= 32'd0;
        frame_count_reg <= 32'd0;
    end else if (axis_beat) begin
        packet_mix_reg <= packet_mix_next;
        byte_count_reg <= byte_count_reg + 1'b1;

        if (s_axis_tlast) begin
            frame_count_reg <= frame_count_reg + 1'b1;
        end
    end
end

endmodule

module de2_eth_frame_rebuild_8 (
    input  wire       clk,
    input  wire       rst,

    input  wire [7:0] s_axis_tdata,
    input  wire       s_axis_tvalid,
    output wire       s_axis_tready,
    input  wire       s_axis_tlast,
    input  wire       s_axis_tuser,

    output wire [7:0] m_axis_tdata,
    output wire       m_axis_tvalid,
    input  wire       m_axis_tready,
    output wire       m_axis_tlast,
    output wire       m_axis_tuser,

    output wire       error_header_early_termination
);

localparam [1:0]
    STATE_READ_HEADER      = 2'd0,
    STATE_WRITE_HEADER     = 2'd1,
    STATE_TRANSFER_PAYLOAD = 2'd2;

localparam [3:0] ETH_HEADER_LEN = 4'd14;

reg [1:0] state_reg = STATE_READ_HEADER;
reg [3:0] hdr_ptr_reg = 4'd0;
reg [3:0] header_len_reg = ETH_HEADER_LEN;
reg header_last_reg = 1'b0;
reg header_user_reg = 1'b0;
reg error_header_early_termination_reg = 1'b0;

reg [47:0] eth_dest_mac_reg = 48'd0;
reg [47:0] eth_src_mac_reg = 48'd0;
reg [15:0] eth_type_reg = 16'd0;

wire write_header_last = hdr_ptr_reg == header_len_reg - 4'd1;
wire read_header = state_reg == STATE_READ_HEADER;
wire write_header = state_reg == STATE_WRITE_HEADER;
wire transfer_payload = state_reg == STATE_TRANSFER_PAYLOAD;

assign s_axis_tready = read_header || (transfer_payload && m_axis_tready);

assign m_axis_tdata = write_header ? header_byte(hdr_ptr_reg) : s_axis_tdata;
assign m_axis_tvalid = write_header || (transfer_payload && s_axis_tvalid);
assign m_axis_tlast = write_header ? header_last_reg && write_header_last : s_axis_tlast;
assign m_axis_tuser = write_header ? header_user_reg && write_header_last : s_axis_tuser;

assign error_header_early_termination = error_header_early_termination_reg;

function [7:0] header_byte;
    input [3:0] offset;
    begin
        case (offset)
            4'd0:  header_byte = eth_dest_mac_reg[47:40];
            4'd1:  header_byte = eth_dest_mac_reg[39:32];
            4'd2:  header_byte = eth_dest_mac_reg[31:24];
            4'd3:  header_byte = eth_dest_mac_reg[23:16];
            4'd4:  header_byte = eth_dest_mac_reg[15:8];
            4'd5:  header_byte = eth_dest_mac_reg[7:0];
            4'd6:  header_byte = eth_src_mac_reg[47:40];
            4'd7:  header_byte = eth_src_mac_reg[39:32];
            4'd8:  header_byte = eth_src_mac_reg[31:24];
            4'd9:  header_byte = eth_src_mac_reg[23:16];
            4'd10: header_byte = eth_src_mac_reg[15:8];
            4'd11: header_byte = eth_src_mac_reg[7:0];
            4'd12: header_byte = eth_type_reg[15:8];
            4'd13: header_byte = eth_type_reg[7:0];
            default: header_byte = 8'd0;
        endcase
    end
endfunction

always @(posedge clk) begin
    error_header_early_termination_reg <= 1'b0;

    if (rst) begin
        state_reg <= STATE_READ_HEADER;
        hdr_ptr_reg <= 4'd0;
        header_len_reg <= ETH_HEADER_LEN;
        header_last_reg <= 1'b0;
        header_user_reg <= 1'b0;
        error_header_early_termination_reg <= 1'b0;
        eth_dest_mac_reg <= 48'd0;
        eth_src_mac_reg <= 48'd0;
        eth_type_reg <= 16'd0;
    end else begin
        case (state_reg)
            STATE_READ_HEADER: begin
                if (s_axis_tvalid && s_axis_tready) begin
                    case (hdr_ptr_reg)
                        4'd0:  eth_dest_mac_reg[47:40] <= s_axis_tdata;
                        4'd1:  eth_dest_mac_reg[39:32] <= s_axis_tdata;
                        4'd2:  eth_dest_mac_reg[31:24] <= s_axis_tdata;
                        4'd3:  eth_dest_mac_reg[23:16] <= s_axis_tdata;
                        4'd4:  eth_dest_mac_reg[15:8] <= s_axis_tdata;
                        4'd5:  eth_dest_mac_reg[7:0] <= s_axis_tdata;
                        4'd6:  eth_src_mac_reg[47:40] <= s_axis_tdata;
                        4'd7:  eth_src_mac_reg[39:32] <= s_axis_tdata;
                        4'd8:  eth_src_mac_reg[31:24] <= s_axis_tdata;
                        4'd9:  eth_src_mac_reg[23:16] <= s_axis_tdata;
                        4'd10: eth_src_mac_reg[15:8] <= s_axis_tdata;
                        4'd11: eth_src_mac_reg[7:0] <= s_axis_tdata;
                        4'd12: eth_type_reg[15:8] <= s_axis_tdata;
                        4'd13: eth_type_reg[7:0] <= s_axis_tdata;
                    endcase

                    if (s_axis_tlast) begin
                        header_len_reg <= hdr_ptr_reg + 4'd1;
                        header_last_reg <= 1'b1;
                        header_user_reg <= s_axis_tuser || hdr_ptr_reg != ETH_HEADER_LEN - 4'd1;
                        error_header_early_termination_reg <= hdr_ptr_reg != ETH_HEADER_LEN - 4'd1;
                        hdr_ptr_reg <= 4'd0;
                        state_reg <= STATE_WRITE_HEADER;
                    end else if (hdr_ptr_reg == ETH_HEADER_LEN - 4'd1) begin
                        header_len_reg <= ETH_HEADER_LEN;
                        header_last_reg <= 1'b0;
                        header_user_reg <= 1'b0;
                        hdr_ptr_reg <= 4'd0;
                        state_reg <= STATE_WRITE_HEADER;
                    end else begin
                        hdr_ptr_reg <= hdr_ptr_reg + 4'd1;
                    end
                end
            end
            STATE_WRITE_HEADER: begin
                if (m_axis_tready) begin
                    if (write_header_last) begin
                        hdr_ptr_reg <= 4'd0;
                        state_reg <= header_last_reg ? STATE_READ_HEADER : STATE_TRANSFER_PAYLOAD;
                    end else begin
                        hdr_ptr_reg <= hdr_ptr_reg + 4'd1;
                    end
                end
            end
            STATE_TRANSFER_PAYLOAD: begin
                if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                    hdr_ptr_reg <= 4'd0;
                    state_reg <= STATE_READ_HEADER;
                end
            end
            default: begin
                state_reg <= STATE_READ_HEADER;
                hdr_ptr_reg <= 4'd0;
            end
        endcase
    end
end

endmodule

module mdio_phy_config #
(
    parameter PHY_ADDR = 5'd0,
    parameter POST_RESET_DELAY = 24'd1250000
)
(
    input  wire clk,
    input  wire rst,
    input  wire start,

    output wire mdc,
    input  wire mdio_i,
    output wire mdio_o,
    output wire mdio_t,

    output wire busy,
    output wire done,
    output wire link_up,
    output wire link_1g,
    output wire full_duplex
);

localparam [4:0]
    STATE_IDLE         = 5'd0,
    STATE_WAIT_1       = 5'd1,
    STATE_ISSUE_R20    = 5'd2,
    STATE_WAIT_R20     = 5'd3,
    STATE_ISSUE_W20    = 5'd4,
    STATE_WAIT_W20     = 5'd5,
    STATE_ISSUE_R27    = 5'd6,
    STATE_WAIT_R27     = 5'd7,
    STATE_ISSUE_W27    = 5'd8,
    STATE_WAIT_W27     = 5'd9,
    STATE_ISSUE_RESET  = 5'd10,
    STATE_WAIT_RESET   = 5'd11,
    STATE_WAIT_2       = 5'd12,
    STATE_ISSUE_R0     = 5'd13,
    STATE_WAIT_R0      = 5'd14,
    STATE_DONE         = 5'd15,
    STATE_ISSUE_R17    = 5'd16,
    STATE_WAIT_R17     = 5'd17;

localparam [4:0]
    PHY_REG_BMCR       = 5'd0,
    PHY_REG_PHY_STATUS = 5'd17,
    PHY_REG_EXT_CTRL   = 5'd20,
    PHY_REG_EXT_STATUS = 5'd27;

// Marvell 88E1111 setup: configure only the FPGA-facing RGMII mode/timing.
// Leave copper-side autonegotiation at the DE2-115 strap defaults.
localparam [15:0]
    PHY_EXT_CTRL_RGMII_DELAYS = 16'h0082,
    PHY_EXT_STATUS_RGMII_MODE = 16'h000b,
    PHY_BMCR_RESET            = 16'h8000;

reg [4:0] state_reg = STATE_IDLE;
reg [23:0] delay_counter_reg = 24'd0;
reg [15:0] reg20_reg = 16'd0;
reg [15:0] reg27_reg = 16'd0;
reg [15:0] phy_status_reg = 16'd0;
reg done_reg = 1'b0;
reg link_up_reg = 1'b0;
reg link_1g_reg = 1'b0;
reg full_duplex_reg = 1'b0;

reg cmd_valid_reg = 1'b0;
reg cmd_read_reg = 1'b0;
reg [4:0] cmd_reg_addr_reg = 5'd0;
reg [15:0] cmd_write_data_reg = 16'd0;

wire cmd_ready;
wire resp_valid;
wire [15:0] resp_read_data;

assign busy = !done_reg;
assign done = done_reg;
assign link_up = link_up_reg;
assign link_1g = link_1g_reg;
assign full_duplex = full_duplex_reg;

mdio_master mdio_master_inst (
    .clk(clk),
    .rst(rst),

    .cmd_valid(cmd_valid_reg),
    .cmd_ready(cmd_ready),
    .cmd_read(cmd_read_reg),
    .cmd_phy_addr(PHY_ADDR),
    .cmd_reg_addr(cmd_reg_addr_reg),
    .cmd_write_data(cmd_write_data_reg),

    .resp_valid(resp_valid),
    .resp_read_data(resp_read_data),

    .mdc(mdc),
    .mdio_i(mdio_i),
    .mdio_o(mdio_o),
    .mdio_t(mdio_t)
);

always @(posedge clk) begin
    cmd_valid_reg <= 1'b0;

    if (rst || !start) begin
        state_reg <= STATE_IDLE;
        delay_counter_reg <= 24'd0;
        reg20_reg <= 16'd0;
        reg27_reg <= 16'd0;
        phy_status_reg <= 16'd0;
        done_reg <= 1'b0;
        link_up_reg <= 1'b0;
        link_1g_reg <= 1'b0;
        full_duplex_reg <= 1'b0;
    end else begin
        case (state_reg)
            STATE_IDLE: begin
                delay_counter_reg <= 24'd0;
                done_reg <= 1'b0;
                state_reg <= STATE_WAIT_1;
            end
            STATE_WAIT_1: begin
                delay_counter_reg <= delay_counter_reg + 1'b1;

                if (delay_counter_reg >= POST_RESET_DELAY) begin
                    delay_counter_reg <= 24'd0;
                    state_reg <= STATE_ISSUE_R20;
                end
            end
            STATE_ISSUE_R20: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b1;
                    cmd_reg_addr_reg <= PHY_REG_EXT_CTRL;
                    cmd_write_data_reg <= 16'd0;
                    state_reg <= STATE_WAIT_R20;
                end
            end
            STATE_WAIT_R20: begin
                if (resp_valid) begin
                    reg20_reg <= resp_read_data | PHY_EXT_CTRL_RGMII_DELAYS;
                    state_reg <= STATE_ISSUE_W20;
                end
            end
            STATE_ISSUE_W20: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b0;
                    cmd_reg_addr_reg <= PHY_REG_EXT_CTRL;
                    cmd_write_data_reg <= reg20_reg;
                    state_reg <= STATE_WAIT_W20;
                end
            end
            STATE_WAIT_W20: begin
                if (resp_valid) begin
                    state_reg <= STATE_ISSUE_R27;
                end
            end
            STATE_ISSUE_R27: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b1;
                    cmd_reg_addr_reg <= PHY_REG_EXT_STATUS;
                    cmd_write_data_reg <= 16'd0;
                    state_reg <= STATE_WAIT_R27;
                end
            end
            STATE_WAIT_R27: begin
                if (resp_valid) begin
                    reg27_reg <= (resp_read_data & 16'hfff0) | PHY_EXT_STATUS_RGMII_MODE;
                    state_reg <= STATE_ISSUE_W27;
                end
            end
            STATE_ISSUE_W27: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b0;
                    cmd_reg_addr_reg <= PHY_REG_EXT_STATUS;
                    cmd_write_data_reg <= reg27_reg;
                    state_reg <= STATE_WAIT_W27;
                end
            end
            STATE_WAIT_W27: begin
                if (resp_valid) begin
                    state_reg <= STATE_ISSUE_RESET;
                end
            end
            STATE_ISSUE_RESET: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b0;
                    cmd_reg_addr_reg <= PHY_REG_BMCR;
                    cmd_write_data_reg <= PHY_BMCR_RESET;
                    delay_counter_reg <= 24'd0;
                    state_reg <= STATE_WAIT_RESET;
                end
            end
            STATE_WAIT_RESET: begin
                if (resp_valid) begin
                    delay_counter_reg <= 24'd0;
                    state_reg <= STATE_WAIT_2;
                end
            end
            STATE_WAIT_2: begin
                delay_counter_reg <= delay_counter_reg + 1'b1;

                if (delay_counter_reg >= POST_RESET_DELAY) begin
                    delay_counter_reg <= 24'd0;
                    state_reg <= STATE_ISSUE_R0;
                end
            end
            STATE_ISSUE_R0: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b1;
                    cmd_reg_addr_reg <= PHY_REG_BMCR;
                    cmd_write_data_reg <= 16'd0;
                    state_reg <= STATE_WAIT_R0;
                end
            end
            STATE_WAIT_R0: begin
                if (resp_valid) begin
                    delay_counter_reg <= 24'd0;
                    if (resp_read_data[15]) begin
                        state_reg <= STATE_WAIT_2;
                    end else begin
                        state_reg <= STATE_DONE;
                    end
                end
            end
            STATE_DONE: begin
                done_reg <= 1'b1;
                delay_counter_reg <= delay_counter_reg + 1'b1;

                if (delay_counter_reg >= POST_RESET_DELAY) begin
                    delay_counter_reg <= 24'd0;
                    state_reg <= STATE_ISSUE_R17;
                end
            end
            STATE_ISSUE_R17: begin
                if (cmd_ready) begin
                    cmd_valid_reg <= 1'b1;
                    cmd_read_reg <= 1'b1;
                    cmd_reg_addr_reg <= PHY_REG_PHY_STATUS;
                    cmd_write_data_reg <= 16'd0;
                    state_reg <= STATE_WAIT_R17;
                end
            end
            STATE_WAIT_R17: begin
                if (resp_valid) begin
                    phy_status_reg <= resp_read_data;
                    link_up_reg <= resp_read_data[10];
                    link_1g_reg <= resp_read_data[15:14] == 2'b10;
                    full_duplex_reg <= resp_read_data[13];
                    state_reg <= STATE_DONE;
                end
            end
            default: begin
                state_reg <= STATE_IDLE;
            end
        endcase
    end
end

endmodule

module mdio_master #
(
    parameter CLK_DIV = 8'd64
)
(
    input  wire        clk,
    input  wire        rst,

    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire        cmd_read,
    input  wire [4:0]  cmd_phy_addr,
    input  wire [4:0]  cmd_reg_addr,
    input  wire [15:0] cmd_write_data,

    output wire        resp_valid,
    output wire [15:0] resp_read_data,

    output wire        mdc,
    input  wire        mdio_i,
    output wire        mdio_o,
    output wire        mdio_t
);

reg busy_reg = 1'b0;
reg mdc_reg = 1'b0;
reg mdio_o_reg = 1'b1;
reg mdio_t_reg = 1'b1;
reg resp_valid_reg = 1'b0;
reg [15:0] resp_read_data_reg = 16'd0;

reg read_cmd_reg = 1'b0;
reg [4:0] phy_addr_reg = 5'd0;
reg [4:0] reg_addr_reg = 5'd0;
reg [15:0] write_data_reg = 16'd0;
reg [5:0] bit_count_reg = 6'd0;
reg phase_reg = 1'b0;
reg [7:0] clk_div_count_reg = 8'd0;

assign cmd_ready = !busy_reg;
assign resp_valid = resp_valid_reg;
assign resp_read_data = resp_read_data_reg;
assign mdc = mdc_reg;
assign mdio_o = mdio_o_reg;
assign mdio_t = mdio_t_reg;

function mdio_bit;
    input [5:0] bit_index;
    input read_cmd;
    input [4:0] phy_addr;
    input [4:0] reg_addr;
    input [15:0] write_data;
    begin
        if (bit_index < 6'd32) begin
            mdio_bit = 1'b1;
        end else if (bit_index == 6'd32) begin
            mdio_bit = 1'b0;
        end else if (bit_index == 6'd33) begin
            mdio_bit = 1'b1;
        end else if (bit_index == 6'd34) begin
            mdio_bit = read_cmd;
        end else if (bit_index == 6'd35) begin
            mdio_bit = !read_cmd;
        end else if (bit_index < 6'd41) begin
            mdio_bit = phy_addr[40-bit_index];
        end else if (bit_index < 6'd46) begin
            mdio_bit = reg_addr[45-bit_index];
        end else if (read_cmd) begin
            mdio_bit = 1'b1;
        end else if (bit_index == 6'd46) begin
            mdio_bit = 1'b1;
        end else if (bit_index == 6'd47) begin
            mdio_bit = 1'b0;
        end else begin
            mdio_bit = write_data[63-bit_index];
        end
    end
endfunction

function mdio_tristate;
    input [5:0] bit_index;
    input read_cmd;
    begin
        mdio_tristate = read_cmd && bit_index >= 6'd46;
    end
endfunction

always @(posedge clk) begin
    resp_valid_reg <= 1'b0;

    if (rst) begin
        busy_reg <= 1'b0;
        mdc_reg <= 1'b0;
        mdio_o_reg <= 1'b1;
        mdio_t_reg <= 1'b1;
        resp_valid_reg <= 1'b0;
        resp_read_data_reg <= 16'd0;
        read_cmd_reg <= 1'b0;
        phy_addr_reg <= 5'd0;
        reg_addr_reg <= 5'd0;
        write_data_reg <= 16'd0;
        bit_count_reg <= 6'd0;
        phase_reg <= 1'b0;
        clk_div_count_reg <= 8'd0;
    end else if (!busy_reg) begin
        mdc_reg <= 1'b0;
        mdio_o_reg <= 1'b1;
        mdio_t_reg <= 1'b1;
        phase_reg <= 1'b0;
        clk_div_count_reg <= 8'd0;

        if (cmd_valid) begin
            busy_reg <= 1'b1;
            read_cmd_reg <= cmd_read;
            phy_addr_reg <= cmd_phy_addr;
            reg_addr_reg <= cmd_reg_addr;
            write_data_reg <= cmd_write_data;
            resp_read_data_reg <= 16'd0;
            bit_count_reg <= 6'd0;
            mdio_o_reg <= 1'b1;
            mdio_t_reg <= 1'b0;
        end
    end else begin
        if (clk_div_count_reg == CLK_DIV-1) begin
            clk_div_count_reg <= 8'd0;

            if (!phase_reg) begin
                mdc_reg <= 1'b1;
                phase_reg <= 1'b1;

                if (read_cmd_reg && bit_count_reg >= 6'd48) begin
                    resp_read_data_reg[63-bit_count_reg] <= mdio_i;
                end
            end else begin
                mdc_reg <= 1'b0;
                phase_reg <= 1'b0;

                if (bit_count_reg == 6'd63) begin
                    busy_reg <= 1'b0;
                    mdio_o_reg <= 1'b1;
                    mdio_t_reg <= 1'b1;
                    resp_valid_reg <= 1'b1;
                end else begin
                    bit_count_reg <= bit_count_reg + 1'b1;
                    mdio_o_reg <= mdio_bit(bit_count_reg + 1'b1, read_cmd_reg, phy_addr_reg, reg_addr_reg, write_data_reg);
                    mdio_t_reg <= mdio_tristate(bit_count_reg + 1'b1, read_cmd_reg);
                end
            end
        end else begin
            clk_div_count_reg <= clk_div_count_reg + 1'b1;
        end
    end
end

endmodule

`resetall
