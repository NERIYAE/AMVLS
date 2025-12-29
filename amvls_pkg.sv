package amvls_pkg;

parameter int unsigned CLK_HZ   = 100_000_000; // 100 MHz
parameter int unsigned TICK_HZ  = 1_000;       // 1 kHz
parameter int unsigned TICK_US  = 1_000;       // 1 ms

  localparam int X_I = 24; localparam int X_F = 8;   localparam int X_W = X_I+X_F; 
  localparam int V_I = 16; localparam int V_F = 16;  localparam int V_W = V_I+V_F;
  localparam int A_I = 8;  localparam int A_F = 24;  localparam int A_W = A_I+A_F; 
  localparam int K_I = 2;  localparam int K_F = 30;  localparam int K_W = K_I+K_F; 

  typedef logic signed [X_W-1:0] q24_8_t;   // position
  typedef logic signed [V_W-1:0] q16_16_t;  // velocity
  typedef logic signed [A_W-1:0] q8_24_t;   // acceleration
  typedef logic signed [K_W-1:0] q2_30_t;   // drag coefficient
  
  // Min/Max limits per type
  localparam q24_8_t  X_MIN = q24_8_t'($signed(-1) <<< (X_W-1));
  localparam q24_8_t  X_MAX = ~X_MIN;
  localparam q16_16_t V_MIN = q16_16_t'($signed(-1) <<< (V_W-1));
  localparam q16_16_t V_MAX = ~V_MIN;
  localparam q8_24_t  A_MIN = q8_24_t'($signed(-1) <<< (A_W-1));
  localparam q8_24_t  A_MAX = ~A_MIN;
  
  typedef enum logic [0:0] {
    CAR_NORMAL = 1'b0,
    CAR_TRUCK  = 1'b1
  } car_type_e;
  
  localparam q2_30_t K_NORMAL = q2_30_t'(32'sd1073742);  // ~0.001 in Q2.30
  localparam q2_30_t K_TRUCK  = q2_30_t'(32'sd2147484);  // ~0.002 in Q2.30
  
  localparam q8_24_t A_CMD_MAX_ACCEL = q8_24_t'(32'sd30      <<< A_F); // 
  localparam q8_24_t A_CMD_MAX_BRAKE = q8_24_t'(-32'sd128     <<< A_F); // 
  
  localparam int TICK_ID_W = 16;
  typedef logic [TICK_ID_W-1:0] tick_id_t;
  
function automatic logic signed [63:0]
  arshift_rn (input logic signed [63:0] x, input int unsigned shift);
    logic signed [63:0] bias;
    if (shift == 0) begin
      return x;
    end
    if (x >= 0) begin
      bias = 64'sd1 <<< (shift-1);
      return (x + bias) >>> shift;
    end
    else begin
      bias = 64'sd1 <<< (shift-1);
      return (x - bias) >>> shift;
    end
  endfunction

function automatic logic signed [63:0]
  rescale_sf (input logic signed [63:0] x, input int in_frac, input int out_frac);
    if (out_frac == in_frac) begin
      return x;
    end 
    else if (out_frac < in_frac) begin
      return arshift_rn(x, in_frac - out_frac);
    end 
    else begin
      return (x <<< (out_frac - in_frac));
    end
  endfunction  

function automatic logic signed [31:0]
  sat_to_w (input logic signed [63:0] in_wide, input int W);
    logic signed [31:0] maxv, minv;
    minv = (32'sd1 <<< (W-1));        // e.g., 0x8000... for width W
    minv = -minv;
    maxv = ~minv;
    if (in_wide > maxv) return maxv;
    if (in_wide < minv) return minv;
    return in_wide[31:0];
  endfunction

function automatic q24_8_t to_q24_8 (input logic signed [63:0] x, input int in_frac);
    to_q24_8 = q24_8_t'(sat_to_w(rescale_sf(x, in_frac, X_F), X_W));
  endfunction  
  
function automatic q16_16_t to_q16_16 (input logic signed [63:0] x, input int in_frac);
    to_q16_16 = q16_16_t'(sat_to_w(rescale_sf(x, in_frac, V_F), V_W));
  endfunction
  
function automatic q8_24_t to_q8_24 (input logic signed [63:0] x, input int in_frac);
    to_q8_24 = q8_24_t'(sat_to_w(rescale_sf(x, in_frac, A_F), A_W));
  endfunction

function automatic q8_24_t clamp_a_cmd (input q8_24_t a_cmd);
    if (a_cmd > A_CMD_MAX_ACCEL) return A_CMD_MAX_ACCEL;
    if (a_cmd < A_CMD_MAX_BRAKE) return A_CMD_MAX_BRAKE;
    return a_cmd;
  endfunction  
  
function automatic logic signed [63:0] v_square_q32_32 (input q16_16_t v);
    logic signed [63:0] v64;
    v64 = {{32{v[31]}}, v}; // sign-extend
    v_square_q32_32 = v64 * v64; // Q16.16 * Q16.16 -> Q32.32
  endfunction  
  
function automatic logic signed [63:0]
  k_mul_vsq_q34_62 (input q2_30_t k, input logic signed [63:0] v_sq_q32_32);
    logic signed [63:0] k64;
    k64 = {{32{k[31]}}, k};
    k_mul_vsq_q34_62 = k64 * v_sq_q32_32; // Q2.30 * Q32.32 -> Q34.62
  endfunction  
  
function automatic q8_24_t drag_q34_62_to_q8_24 (input logic signed [63:0] k_v2_q34_62);
    drag_q34_62_to_q8_24 = to_q8_24(k_v2_q34_62, 62);
  endfunction 
 
function automatic logic signed [63:0]
  scale_div_tick (input logic signed [63:0] x_wide);
    localparam logic [63:0] RECIP_TICK_Q32 = 64'd4294967; // round(2^32 / 1000)
    scale_div_tick = arshift_rn( (x_wide * $signed(RECIP_TICK_Q32)), 32 );
  endfunction

function automatic logic signed [1:0] sgn2 (input q16_16_t v);
    if (v > 0) return 2'sd1;
    if (v < 0) return -2'sd1;
    return 2'sd0;
  endfunction  

function automatic q16_16_t clamp_v_forward (input q16_16_t v_in);
  if (v_in < q16_16_t'(32'sd0)) return q16_16_t'(32'sd0);
  else return v_in;
endfunction  

endpackage
