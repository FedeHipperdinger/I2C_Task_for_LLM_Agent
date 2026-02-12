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

  i2c_state_e st;

  logic engine_en;
  logic tick_r, tick_f;

  logic sda_drive_low;

  logic load_tx, tx_enable, rx_enable;
  logic [7:0] tx_byte;
  logic [7:0] rx_byte;
  logic tx_bit;
  logic tx_done, rx_done;

  logic [7:0] bytes_total;
  logic [7:0] bytes_rcvd;

  logic [7:0] addr_frame;

  i2c_bit_engine #(.CLK_DIV(CLK_DIV)) u_eng (
    .clk(clk), .rst_n(rst_n),
    .enable(engine_en),
    .sda_drive_low(sda_drive_low),
    .scl(scl),
    .tick_scl_rise(tick_r),
    .tick_scl_fall(tick_f),
    .sda_o(sda_o),
    .sda_oe(sda_oe)
  );

  i2c_shift u_shift (
    .clk(clk), .rst_n(rst_n),
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

  // defaults
  always_comb begin
    engine_en     = (st != ST_IDLE);
    busy          = (st != ST_IDLE) && (st != ST_DONE);
    done          = 1'b0;

    load_tx       = 1'b0;
    tx_enable     = 1'b0;
    rx_enable     = 1'b0;
    tx_byte       = 8'h00;

    // Default: release SDA unless explicitly driving low
    sda_drive_low = 1'b0;

    unique case (st)
      ST_IDLE: begin
        engine_en = 1'b0;
      end

      // START: ensure SDA goes low while SCL is high
      ST_START: begin
        // pull SDA low immediately (engine holds SCL high at enable start)
        sda_drive_low = 1'b1;
      end

      ST_ADDR: begin
        // shift address byte out; change data on fall, stable on high
        tx_enable = 1'b1;
        // open-drain mapping
        sda_drive_low = (tx_bit == 1'b0);
      end

      ST_ADDR_ACK: begin
        // IMPORTANT: release SDA so slave can ACK/NACK
        sda_drive_low = 1'b0;
      end

      ST_READ: begin
        // release SDA for slave-driven data bits
        rx_enable     = 1'b1;
        sda_drive_low = 1'b0;
      end

      ST_READ_ACK: begin
        // Master drives ACK=0 for all but last byte, then NACK=1 (release)
        if (bytes_rcvd < (bytes_total - 1)) begin
          // ACK
          sda_drive_low = 1'b1;
        end else begin
          // last byte: NACK
          sda_drive_low = 1'b0;
        end
      end

      ST_STOP: begin
        // STOP: SDA low->high while SCL high.
        // Ensure SDA is released at STOP time; we rely on timing in seq block.
        sda_drive_low = 1'b0;
      end

      ST_DONE: begin
        done = 1'b1;
      end

      default: ;
    endcase
  end

  // Sequencing / pulses
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st         <= ST_IDLE;
      ack_error  <= 1'b0;
      addr_frame <= 8'h00;
      bytes_total<= 8'd0;
      bytes_rcvd <= 8'd0;
      rvalid     <= 1'b0;
      rdata      <= 8'h00;
      done       <= 1'b0;
    end else begin
      rvalid <= 1'b0;
      done   <= 1'b0;

      if (cmd_start) begin
        ack_error <= 1'b0; // clear on new command
      end

      unique case (st)
        ST_IDLE: begin
          if (cmd_start) begin
            addr_frame  <= {cmd_addr, 1'b1};
            bytes_total <= (cmd_len == 0) ? 8'd1 : cmd_len;
            bytes_rcvd  <= 8'd0;
            st          <= ST_START;
          end
        end

        ST_START: begin
          // Wait for first SCL falling edge after enabling engine,
          // then load address and begin shifting.
          if (tick_f) begin
            st <= ST_ADDR;
          end
        end

        ST_ADDR: begin
          // Load address at the first tick_f inside ST_ADDR
          if (tick_f && (bytes_rcvd == 0)) begin
            // use bytes_rcvd==0 as a convenient "loaded" gate (safe here)
            // pulse load_tx via comb below (see assigns)
          end

          if (tx_done) begin
            st <= ST_ADDR_ACK;
          end
        end

        ST_ADDR_ACK: begin
          // Sample ACK during SCL high; use rising edge marker
          if (tick_r) begin
            if (sda_i == 1'b1) begin
              ack_error <= 1'b1;
              st <= ST_STOP;
            end else begin
              st <= ST_READ;
            end
          end
        end

        ST_READ: begin
          if (rx_done) begin
            rdata  <= rx_byte;
            rvalid <= 1'b1;
            bytes_rcvd <= bytes_rcvd + 8'd1;
            st <= ST_READ_ACK;
          end
        end

        ST_READ_ACK: begin
          // After ACK/NACK bit completes, proceed
          if (tick_r) begin
            if (bytes_rcvd >= bytes_total) begin
              st <= ST_STOP;
            end else begin
              st <= ST_READ;
            end
          end
        end

        ST_STOP: begin
          // Wait for an SCL high point, then finish
          if (tick_r) begin
            st   <= ST_DONE;
          end
        end

        ST_DONE: begin
          done <= 1'b1;
          st   <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  // Clean shifter load: pulse exactly once at entry to ST_ADDR
  logic addr_loaded;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) addr_loaded <= 1'b0;
    else begin
      if (st == ST_IDLE) addr_loaded <= 1'b0;
      else if (st == ST_ADDR && tick_f) addr_loaded <= 1'b1;
    end
  end

  assign load_tx = (st == ST_ADDR) && tick_f && !addr_loaded;
  assign tx_byte = addr_frame;

endmodule

