`timescale 1ns/1ps

module i2c_shift(
  input  logic clk,
  input  logic rst_n,

  input  logic tick_scl_rise,
  input  logic tick_scl_fall,

  input  logic sda_i,

  // control
  input  logic load_tx,
  input  logic [7:0] tx_byte,
  input  logic tx_enable,   // shift out on SCL-fall boundaries
  input  logic rx_enable,   // sample in on SCL-??? (baseline uses FALL = bug)

  output logic [7:0] rx_byte,
  output logic       rx_done,

  // TX bit to drive (controller will translate to open-drain)
  output logic       tx_bit,
  output logic       tx_done
);

  logic [7:0] tx_sh;
  logic [7:0] rx_sh;
  logic [2:0] bit_idx;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_sh   <= 8'h00;
      rx_sh   <= 8'h00;
      bit_idx <= 3'd0;
      tx_bit  <= 1'b1;
      tx_done <= 1'b0;
      rx_done <= 1'b0;
      rx_byte <= 8'h00;
    end else begin
      tx_done <= 1'b0;
      rx_done <= 1'b0;

      if (load_tx) begin
        tx_sh   <= tx_byte;
        bit_idx <= 3'd0;
        tx_bit  <= tx_byte[7];
      end

      // TX: update bit on SCL falling edge (legal: data changes while SCL low)
      if (tx_enable && tick_scl_fall) begin
        tx_sh <= {tx_sh[6:0], 1'b0};
        bit_idx <= bit_idx + 3'd1;
        tx_bit <= tx_sh[6];
        if (bit_idx == 3'd7) begin
          tx_done <= 1'b1; // 8 bits sent
        end
      end

      // RX: BASELINE BUG — sample on falling edge (should be rising edge)
      if (rx_enable && tick_scl_fall) begin
        rx_sh <= {rx_sh[6:0], sda_i};
        bit_idx <= bit_idx + 3'd1;
        if (bit_idx == 3'd7) begin
          rx_done <= 1'b1;
          rx_byte <= {rx_sh[6:0], sda_i};
        end
      end
    end
  end

endmodule
