package i2c_pkg;
  parameter int unsigned CLK_DIV = 50;

  typedef enum logic [2:0] {
    ST_IDLE      = 3'd0,
    ST_START     = 3'd1,
    ST_ADDR      = 3'd2,
    ST_ADDR_ACK  = 3'd3,
    ST_READ      = 3'd4,
    ST_READ_ACK  = 3'd5,
    ST_STOP      = 3'd6,
    ST_DONE      = 3'd7
  } i2c_state_e;
endpackage

