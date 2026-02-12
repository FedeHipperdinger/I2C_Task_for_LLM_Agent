`timescale 1ns/1ps

module i2c_bit_engine #(
  parameter int unsigned CLK_DIV = 50
)(
  input  logic clk,
  input  logic rst_n,

  input  logic enable,

  input  logic sda_drive_low,

  output logic scl,
  output logic tick_scl_rise,
  output logic tick_scl_fall,

  output logic sda_o,
  output logic sda_oe
);

  logic [$clog2(CLK_DIV)-1:0] div_cnt;
  logic scl_q;

  assign sda_o  = 1'b0;
  assign sda_oe = enable && sda_drive_low;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div_cnt       <= '0;
      scl_q         <= 1'b1;
      tick_scl_rise <= 1'b0;
      tick_scl_fall <= 1'b0;
    end else begin
      tick_scl_rise <= 1'b0;
      tick_scl_fall <= 1'b0;

      if (!enable) begin
        div_cnt <= '0;
        scl_q   <= 1'b1;
      end else begin
        if (div_cnt == CLK_DIV-1) begin
          div_cnt <= '0;
          scl_q   <= ~scl_q;
          if (~scl_q) tick_scl_rise <= 1'b1;
          else        tick_scl_fall <= 1'b1;
        end else begin
          div_cnt <= div_cnt + 1'b1;
        end
      end
    end
  end

  assign scl = scl_q;

endmodule

