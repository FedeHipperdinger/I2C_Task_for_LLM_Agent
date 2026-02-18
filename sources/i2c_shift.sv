`timescale 1ns/1ps

module i2c_shift(
  input  logic clk,
  input  logic rst_n,
  input  logic tick_scl_rise,
  input  logic tick_scl_fall,
  input  logic sda_i,

  input  logic       load_tx,
  input  logic [7:0] tx_byte,
  input  logic       tx_enable,
  input  logic       rx_enable,

  output logic [7:0] rx_byte,
  output logic       rx_done,
  output logic       tx_bit,
  output logic       tx_done
);

  // TODO: implement TX shift (MSB-first), RX sampling, and done pulses.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_byte <= 8'h00;
      rx_done <= 1'b0;
      tx_bit  <= 1'b1;
      tx_done <= 1'b0;
    end else begin
      rx_byte <= 8'h00;
      rx_done <= 1'b0;
      tx_bit  <= 1'b1;
      tx_done <= 1'b0;
    end
  end

endmodule
