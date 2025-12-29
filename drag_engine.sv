
module drag_engine (
  input  amvls_pkg::q16_16_t v_in,        // signed velocity (Q16.16)
  input  amvls_pkg::q2_30_t  k_in,        // drag coefficient K (Q2.30)
  output amvls_pkg::q8_24_t  a_drag_mag   // drag acceleration magnitude (Q8.24, >=0)
);
  import amvls_pkg::*;

  logic signed [63:0] v_sq_q32_32;   // v^2 as Q32.32
  logic signed [63:0] kv2_q34_62;    // K·v^2 as Q34.62

  q8_24_t a_drag_mag_int;


  always_comb begin
    v_sq_q32_32 = v_square_q32_32(v_in);
    kv2_q34_62  = k_mul_vsq_q34_62(k_in, v_sq_q32_32);
    a_drag_mag_int = drag_q34_62_to_q8_24(kv2_q34_62);
  end

  assign a_drag_mag = a_drag_mag_int;

endmodule
