
module accel_combiner (
  input  amvls_pkg::q8_24_t  a_cmd_in,        // commanded accel (Q8.24)
  input  amvls_pkg::q8_24_t  a_drag_mag_in,   // |a_drag| from drag_engine (Q8.24, >=0)
  input  amvls_pkg::q16_16_t v_in,            // velocity (Q16.16) for drag direction
  output amvls_pkg::q8_24_t  a_eff_out        // effective accel (Q8.24)
);
  import amvls_pkg::*;

  q8_24_t a_cmd_clamped;
  q8_24_t a_eff_int;

  logic signed [1:0] v_sgn;
  logic signed [63:0] a_cmd64;
  logic signed [63:0] drag_mag64;
  logic signed [63:0] sgn64;
  logic signed [63:0] a_drag64;
  logic signed [63:0] a_sum64;

 
  always_comb begin
    a_cmd_clamped = clamp_a_cmd(a_cmd_in);
    v_sgn = sgn2(v_in);
    a_cmd64    = {{(64-A_W){a_cmd_clamped[A_W-1]}},   a_cmd_clamped};
    drag_mag64 = {{(64-A_W){a_drag_mag_in[A_W-1]}},   a_drag_mag_in};
    sgn64 = {{62{v_sgn[1]}}, v_sgn};
    a_drag64 = drag_mag64 * sgn64;
    a_sum64 = a_cmd64 - a_drag64;
    a_eff_int = to_q8_24(a_sum64, A_F);
  end

  assign a_eff_out = a_eff_int;
    //assign a_eff_out = 32'shF000_0000;

endmodule
