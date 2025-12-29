
module kinematics_integrator (
  tick_if.consumer     tick,      // tick sideband (clk, rst_n, tick_en, tick_id)
  input  amvls_pkg::q8_24_t  a_eff_in, // effective accel (Q8.24)
  output amvls_pkg::q16_16_t v_out,    // velocity state (Q16.16)
  output amvls_pkg::q24_8_t  x_out     // position state (Q24.8)
);
  import amvls_pkg::*;

  q16_16_t v_reg;  // velocity
  q24_8_t  x_reg;  // position

  logic signed [63:0] a_eff_64;
  logic signed [63:0] dv_q8_24_64;
  logic signed [63:0] v_reg_64;
  logic signed [63:0] dv_q16_16_64;
  logic signed [63:0] v_sum_64;
  logic signed [63:0] v_for_dx_64;
  logic signed [63:0] dx_q16_16_64;
  logic signed [63:0] x_reg_64;
  logic signed [63:0] dx_q24_8_64;
  logic signed [63:0] x_sum_64;

  q16_16_t dv_q16_16;
  q16_16_t v_next;
  q24_8_t  dx_q24_8;

  always_ff @(posedge tick.clk_100m or negedge tick.rst_n) begin
    if (!tick.rst_n) begin
      v_reg <= '0;
      x_reg <= '0;
    end
    else begin
      if (tick.tick_en) begin
        a_eff_64     = {{(64-A_W){a_eff_in[A_W-1]}}, a_eff_in};
        dv_q8_24_64  = scale_div_tick(a_eff_64);
        dv_q16_16    = to_q16_16(dv_q8_24_64, A_F);

        v_reg_64     = {{(64-V_W){v_reg[V_W-1]}}, v_reg};
        dv_q16_16_64 = {{(64-V_W){dv_q16_16[V_W-1]}}, dv_q16_16};
        v_sum_64     = v_reg_64 + dv_q16_16_64;

        v_next       = to_q16_16(v_sum_64, V_F);
        v_next       = clamp_v_forward(v_next);
        v_reg        <= v_next;

        v_for_dx_64  = {{(64-V_W){v_next[V_W-1]}}, v_next};
        dx_q16_16_64 = scale_div_tick(v_for_dx_64);
        dx_q24_8     = to_q24_8(dx_q16_16_64, V_F);

        x_reg_64     = {{(64-X_W){x_reg[X_W-1]}}, x_reg};
        dx_q24_8_64  = {{(64-X_W){dx_q24_8[X_W-1]}}, dx_q24_8};
        x_sum_64     = x_reg_64 + dx_q24_8_64;
        x_reg        <= to_q24_8(x_sum_64, X_F);
      end
      else begin
        v_reg <= v_reg;
        x_reg <= x_reg;
      end
    end
  end

  assign v_out = v_reg;
  assign x_out = x_reg;

endmodule
