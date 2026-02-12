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
  logic       stop_phase;
  logic       start_wait;

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
    // default engine enable; may be overridden in specific states
    engine_en     = (st != ST_IDLE);
    busy          = (st != ST_IDLE) && (st != ST_DONE);
    done          = 1'b0;

    tx_enable     = 1'b0;
    rx_enable     = 1'b0;

    // Default: release SDA unless explicitly driving low
    sda_drive_low = 1'b0;

    unique case (st)
      ST_IDLE: begin
        engine_en = 1'b0;
      end

      // START: ensure SDA goes low while SCL is high
      // The bit engine starts with SCL high when enable is first asserted.
      // We wait one cycle (start_wait) to ensure SCL is stable high and
      // the testbench can sample SDA=1, then drive SDA low to create
      // the START condition (SDA 1->0 while SCL high).
      ST_START: begin
        if (start_wait) begin
          // Drive SDA low while SCL is high to create START condition
          sda_drive_low = 1'b1;
        end
      end

      ST_ADDR: begin
        // shift address byte out; change data on fall, stable on high
        // Only start shifting once the address byte has been loaded.
        tx_enable = addr_loaded;
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
        // bytes_rcvd counts bytes already received; if more remain,
        // ACK (drive low). For the final byte (bytes_rcvd == bytes_total),
        // NACK by releasing SDA.
        if (bytes_rcvd < bytes_total) begin
          // ACK: more bytes to come
          sda_drive_low = 1'b1;
        end else begin
          // Last byte: NACK
          sda_drive_low = 1'b0;
        end
      end

      ST_STOP: begin
        // For STOP we force the bit engine off so SCL is held high,
        // and generate SDA low->high while SCL is high.
        engine_en = 1'b0;
        // While stop_phase==0 we actively drive SDA low; once stop_phase==1
        // we release SDA so the line rises while SCL is high.
        sda_drive_low = (stop_phase == 1'b0);
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
      stop_phase <= 1'b0;
      start_wait <= 1'b0;
    end else begin
      rvalid <= 1'b0;
      done   <= 1'b0;

      if (cmd_start) begin
        ack_error <= 1'b0; // clear on new command
      end

      // Reset start_wait when leaving ST_START
      if (st != ST_START) begin
        start_wait <= 1'b0;
      end

      unique case (st)
        ST_IDLE: begin
          if (cmd_start) begin
            addr_frame  <= {cmd_addr, 1'b1};
            bytes_total <= (cmd_len == 0) ? 8'd1 : cmd_len;
            bytes_rcvd  <= 8'd0;
            st          <= ST_START;
            stop_phase  <= 1'b0;
          end
        end

        ST_START: begin
          // First cycle: set start_wait to ensure SCL is stable high
          // and testbench can sample SDA=1 before we drive it low.
          if (!start_wait) begin
            start_wait <= 1'b1;
          end else begin
            // After start_wait, we drive SDA low (in comb logic) to create
            // START condition. Wait for first SCL falling edge, then begin
            // address transmission.
            if (tick_f) begin
              st <= ST_ADDR;
            end
          end
        end

        ST_ADDR: begin
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
          // Generate a proper STOP condition with SCL held high:
          //  - First cycle in ST_STOP: drive SDA low (stop_phase==0).
          //  - Second cycle: release SDA to create a 0->1 edge while SCL is high,
          //    then move to ST_DONE.
          if (!stop_phase) begin
            stop_phase <= 1'b1;
          end else begin
            st         <= ST_DONE;
            stop_phase <= 1'b0;
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

  // Track when the address byte has been loaded into the shifter
  logic addr_loaded;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) addr_loaded <= 1'b0;
    else begin
      if (st == ST_IDLE) addr_loaded <= 1'b0;
      else if (load_tx)  addr_loaded <= 1'b1;
    end
  end

  // Pulse load_tx once when we first enter ST_ADDR; shifting only starts
  // afterwards (gated by addr_loaded) so the MSB is stable for the first
  // SCL rising edge when the slave samples it.
  assign load_tx = (st == ST_ADDR) && !addr_loaded;
  assign tx_byte = addr_frame;

endmodule

