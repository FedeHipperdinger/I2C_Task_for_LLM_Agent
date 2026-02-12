`timescale 1ns/1ps

module i2c_shift(
  input  logic clk,
  input  logic rst_n,

  input  logic tick_scl_rise,
  input  logic tick_scl_fall,

  input  logic sda_i,

  input  logic load_tx,
  input  logic [7:0] tx_byte,
  input  logic tx_enable,
  input  logic rx_enable,

  output logic [7:0] rx_byte,
  output logic       rx_done,

  output logic       tx_bit,
  output logic       tx_done
);

  logic [7:0] tx_sh;
  logic [7:0] rx_sh;
  logic [2:0] tx_idx;
  logic [2:0] rx_idx;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_sh   <= 8'h00;
      rx_sh   <= 8'h00;
      tx_idx  <= 3'd0;
      rx_idx  <= 3'd0;
      tx_bit  <= 1'b1;
      tx_done <= 1'b0;
      rx_done <= 1'b0;
      rx_byte <= 8'h00;
    end else begin
      tx_done <= 1'b0;
      rx_done <= 1'b0;

      if (load_tx) begin
        tx_sh  <= tx_byte;
        tx_idx <= 3'd0;
        tx_bit <= tx_byte[7];
      end

      if (tx_enable && tick_scl_fall) begin
        tx_sh  <= {tx_sh[6:0], 1'b0};
        tx_idx <= tx_idx + 3'd1;
        tx_bit <= tx_sh[6];
        if (tx_idx == 3'd7) tx_done <= 1'b1;
      end

      // GOLDEN: sample on SCL rising edge
      if (rx_enable && tick_scl_rise) begin
        rx_sh  <= {rx_sh[6:0], sda_i};
        rx_idx <= rx_idx + 3'd1;
        if (rx_idx == 3'd7) begin
          rx_done <= 1'b1;
          rx_byte <= {rx_sh[6:0], sda_i};
        end
      end

      // reset rx_idx when rx_enable deasserts (byte boundary)
      if (!rx_enable) begin
        rx_idx <= 3'd0;
      end
    end
  end

endmodule

