`timescale 1ns/1ps
import i2c_pkg::*;

module i2c_master_top #(
  parameter int unsigned CLK_DIV = i2c_pkg::CLK_DIV
)(
  input  logic clk,
  input  logic rst_n,

  input  logic        cmd_start,
  input  logic [6:0]  cmd_addr,
  input  logic [7:0]  cmd_len,

  output logic        busy,
  output logic        done,
  output logic        ack_error,
  output logic [7:0]  rdata,
  output logic        rvalid,

  output logic        scl,
  output logic        sda_o,
  output logic        sda_oe,
  input  logic        sda_i
);

  logic engine_en;
  logic tick_r, tick_f;
  logic sda_drive_low;

  logic load_tx, tx_enable, rx_enable;
  logic [7:0] tx_byte;
  logic [7:0] rx_byte;
  logic tx_bit;
  logic tx_done, rx_done;

  // TODO(section 4): implement transaction FSM in this module.
  // Keep helper submodules wired so candidates can focus on control first.
  i2c_bit_engine #(.CLK_DIV(CLK_DIV)) u_eng (
    .clk(clk),
    .rst_n(rst_n),
    .enable(engine_en),
    .sda_drive_low(sda_drive_low),
    .scl(scl),
    .tick_scl_rise(tick_r),
    .tick_scl_fall(tick_f),
    .sda_o(sda_o),
    .sda_oe(sda_oe)
  );

  i2c_shift u_shift (
    .clk(clk),
    .rst_n(rst_n),
    .tick_scl_rise(tick_r),
    .tick_scl_fall(tick_f),
    .sda_i(sda_i),
    .load_tx(load_tx),
    .tx_byte(tx_byte),
    .tx_enable(tx_enable),
    .rx_enable(rx_enable),
    .rx_byte(rx_byte),
    .rx_done(rx_done),
    .tx_bit(tx_bit),
    .tx_done(tx_done)
  );

  always_comb begin
    busy         = 1'b0;
    done         = 1'b0;
    ack_error    = 1'b0;
    rdata        = 8'h00;
    rvalid       = 1'b0;
    engine_en    = 1'b0;
    sda_drive_low= 1'b0;
    load_tx      = 1'b0;
    tx_enable    = 1'b0;
    rx_enable    = 1'b0;
    tx_byte      = 8'h00;

    // Avoid unused-input warnings in starter baseline.
    if (cmd_start || (cmd_addr != 7'h00) || (cmd_len != 8'h00) || sda_i) begin
      busy = 1'b0;
    end
  end

endmodule
