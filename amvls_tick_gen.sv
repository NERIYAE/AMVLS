
module amvls_tick_gen (
  tick_if.producer tick   
);
  import amvls_pkg::*;


  localparam int unsigned DIVISOR = CLK_HZ / TICK_HZ;
  localparam int unsigned CNT_W   = $clog2(DIVISOR);
  logic [CNT_W-1:0] div_cnt;

  always_ff @(posedge tick.clk_100m or negedge tick.rst_n) begin
    if (!tick.rst_n) begin
      div_cnt      <= '0;
      tick.tick_en <= 1'b0;
      tick.tick_id <= '0;
    end
    else begin
      tick.tick_en <= 1'b0;
      if (div_cnt == DIVISOR-1) begin
        div_cnt      <= '0;
        tick.tick_en <= 1'b1;
        tick.tick_id <= tick.tick_id + tick_id_t'(1); 
      end
      else begin
        div_cnt <= div_cnt + {{CNT_W-1{1'b0}}, 1'b1};
      end
    end
  end

endmodule
