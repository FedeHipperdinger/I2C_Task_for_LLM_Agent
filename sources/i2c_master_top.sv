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

  // shift unit
  logic load_tx, tx_enable, rx_enable;
  logic [7:0] tx_byte;
  logic [7:0] rx_byte;
  logic tx_bit;
  logic tx_done, rx_done;

  logic [7:0] bytes_left;
  logic [7:0] addr_frame;

  // Instantiate bit engine
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

  // Instantiate shifter
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

  // Open drain mapping for TX bits: drive low for '0', release for '1'
  // NOTE: during RX phases we should release.
  // BASELINE BUGS are introduced in state control below.
  logic want_drive_low;

  always_comb begin
    // defaults
    engine_en      = (st != ST_IDLE);
    busy           = (st != ST_IDLE) && (st != ST_DONE);
    done           = 1'b0;
    rvalid         = 1'b0;
    rdata          = rx_byte;

    load_tx        = 1'b0;
    tx_enable      = 1'b0;
    rx_enable      = 1'b0;
    tx_byte        = 8'h00;

    want_drive_low = 1'b0;

    unique case (st)
      ST_IDLE: begin
        engine_en = 1'b0;
        busy      = 1'b0;
      end

      // START: generate SDA falling while SCL high
      // We will force SDA low while engine idle-high SCL, then proceed.
      ST_START: begin
        // Pull SDA low immediately (safe because SCL is high at engine start)
        want_drive_low = 1'b1;
      end

      ST_ADDR: begin
        // send address + READ bit
        tx_enable = 1'b1;
        // drive low if tx_bit==0
        want_drive_low = (tx_bit == 1'b0);
      end

      ST_ADDR_ACK: begin
        // BASELINE BUG #2: do NOT release SDA for ACK; mistakenly keep driving low
        // Correct would be want_drive_low=0 during ACK bit.
        want_drive_low = 1'b1;

        // sample ACK on SCL high (we'll sample on rising edge in sequential block)
      end

      ST_READ: begin
        // release SDA during data bits
        rx_enable = 1'b1;
        want_drive_low = 1'b0;
      end

      ST_READ_ACK: begin
        // BASELINE BUG #1: always ACK (drive low), never NACK last byte
        want_drive_low = 1'b1;
      end

      ST_STOP: begin
        // ensure SDA low, then release while SCL high to form STOP
        want_drive_low = 1'b0; // baseline: may be wrong edge; but kept simple
      end

      ST_DONE: begin
        done = 1'b1;
      end

      default: ;
    endcase

    sda_drive_low = want_drive_low;
  end

  // State machine / control sequencing
  logic ack_sampled;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st         <= ST_IDLE;
      ack_error  <= 1'b0;
      bytes_left <= 8'd0;
      addr_frame <= 8'h00;
      ack_sampled<= 1'b0;
    end else begin
      // clear done/rvalid are combinational; ack_error sticky until cmd_start
      if (cmd_start) begin
        ack_error <= 1'b0;
      end

      unique case (st)
        ST_IDLE: begin
          if (cmd_start) begin
            // prepare address frame: 7-bit addr + R/W=1
            addr_frame <= {cmd_addr, 1'b1};
            bytes_left <= (cmd_len == 0) ? 8'd1 : cmd_len;
            st         <= ST_START;
          end
        end

        ST_START: begin
          // after first falling edge of SCL, start shifting address
          if (tick_f) begin
            st <= ST_ADDR;
          end
          // load address once at entry
          if (st == ST_START) begin
            // one-shot load (best-effort)
          end
        end

        ST_ADDR: begin
          // load tx at first entry
          if (!ack_sampled) begin
            // misuse ack_sampled as "loaded" flag for simplicity
            // (baseline skeleton)
            ack_sampled <= 1'b1;
          end
          // ensure shifter has tx_byte loaded near start
          // load_tx is combinational, so do it here with registered pulse:
          // We'll do a cheap pulse when tick_f first occurs after START.
          if (tick_f && ack_sampled) begin
            // nothing
          end

          // When address byte shifted out (8 bits), go to ACK phase
          if (tx_done) begin
            st <= ST_ADDR_ACK;
            ack_sampled <= 1'b0;
          end
        end

        ST_ADDR_ACK: begin
          // sample ACK on SCL rising edge
          if (tick_r && !ack_sampled) begin
            ack_sampled <= 1'b1;
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
            // emit rvalid via combinational in next cycle; also update counter
            bytes_left <= bytes_left - 8'd1;
            st <= ST_READ_ACK;
          end
        end

        ST_READ_ACK: begin
          // after ACK bit completes (use tick_r as phase marker)
          if (tick_r) begin
            if (bytes_left == 0) begin
              st <= ST_STOP;
            end else begin
              st <= ST_READ;
            end
          end
        end

        ST_STOP: begin
          // after a couple of edges, go done
          if (tick_r) begin
            st <= ST_DONE;
          end
        end

        ST_DONE: begin
          st <= ST_IDLE;
          ack_sampled <= 1'b0;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  // Properly pulse load_tx at start of ST_ADDR (baseline minimalism)
  // and pulse rvalid when rx_done
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // nothing
    end else begin
      // load address frame into shifter right when entering ST_ADDR
      if (st == ST_START && tick_f) begin
        // load tx_byte on next cycle via shifter load input (combinational) is too late,
        // so we drive internal reg instead (simplify): use tx_byte direct through load_tx pulse.
      end
    end
  end

  // Hacky but functional: drive shifter load via continuous assign from state
  // (This is okay for baseline; golden will clean this up.)
  assign load_tx = (st == ST_ADDR) && (tick_f); // pulse early-ish
  assign tx_byte = addr_frame;

  // rvalid pulse (on rx_done)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rvalid <= 1'b0;
      rdata  <= 8'h00;
    end else begin
      rvalid <= 1'b0;
      if (rx_done && st == ST_READ) begin
        rdata  <= rx_byte;
        rvalid <= 1'b1;
      end
    end
  end

  // done pulse when leaving ST_DONE
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (st == ST_DONE) done <= 1'b1;
    end
  end

endmodule
